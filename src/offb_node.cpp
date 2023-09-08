#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

#include <offboardholy/PTStates.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_local_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_local_pos = *msg;
}

struct sls_state {
    double x, y, z, alpha, beta, vx, vy, vz, gamma_alpha, gamma_beta;
    // /mavros/local_position/pose
}sls_state1;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher attitude_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    // ros::Subscriber quad_state_sub = nh.subscribe<>


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    

    // ros::Subscriber 

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    mavros_msgs::AttitudeTarget attitude;
    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map"; 
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 0;
    attitude.thrust = 0.8;

    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        attitude.header.stamp = ros::Time::now();
        attitude_setpoint_pub.publish(attitude);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

	mavros_msgs::SetMode land_mode;
	land_mode.request.custom_mode = "AUTO.LAND";

    ros::Time last_request = ros::Time::now();

	double distance = 0;

    int stage = 0;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // if((ros::Time::now() - last_request < ros::Duration(20.0))&& stage==0){
        //     pose.header.stamp = ros::Time::now();
        //     local_pos_pub.publish(pose);
        //     ROS_INFO("Position Control armed");
        // }else{
        //     stage += 1;
        // }

      
        attitude.header.stamp = ros::Time::now();
        attitude_setpoint_pub.publish(attitude);
        // ROS_INFO("Attitude Control armed");


		distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
		+ std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
		+ std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);

		if(distance < 0.1 && (ros::Time::now() - last_request > ros::Duration(50.0)) ){
			if( set_mode_client.call(land_mode) &&
                land_mode.response.mode_sent){
                ROS_INFO("Land enabled");
				break;
            }
		}
		
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
