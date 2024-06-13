/*
About this file:
offb_node.cpp code deals with the off-board control i.e. to be able to control the PX4 flight stack using software running outside of the autopilot.
It is written with MAVROS, PX4 Pro Flight Stack and tested in Gazebo Classic SITL
*/

/* Includes necessary headers for ROS, geometry_msgs, mavros_msgs, and other custom headers. */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

#include <offboardholy/PTStates.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
// #include <gazebo_msgs/LinkStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

/* Includes the custom headers for the control functions. */
#include <StabController.h>
#include <TracController.h>
#include <rtwtypes.h>

/* Includes necessary headers for C++ standard library */
#include <cstddef>
#include <cstdlib>
#include <iostream>

/* Declaring the functions defined below */
void PT_state_pub(ros::Publisher &sls_state_pub);
void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);

/* Global variables to store the data */
geometry_msgs::Pose quadpose;
geometry_msgs::Pose loadpose;
geometry_msgs::Pose pendpose;
geometry_msgs::Twist pendtwist;
geometry_msgs::Twist quadtwist;
geometry_msgs::Twist loadtwist;

mavros_msgs::State current_state;
/* Callback function to save the current state of the autopilot */
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_local_pos;
/* Callback function to save the current position of drone */
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_local_pos = *msg;
}

geometry_msgs::TwistStamped current_local_vel;
/* Callback function to save the current velocity of drone */
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_local_vel = *msg;
}

mavros_msgs::AttitudeTarget attitude;
/* Callback function to save the attitude target of the autopilot */
void attitude_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    attitude = *msg;
}

offboardholy::PTStates PTState;
/* Callback function to update the PTState */
void sls_state_cb(const offboardholy::PTStates::ConstPtr &msg)
{
    PTState = *msg;
}

int main(int argc, char **argv)
{
    /* Initialise ROS Node */
    ros::init(argc, argv, "offb_node");

    /* NodeHandle - Main access point for communications with ROS*/
    ros::NodeHandle nh;

    /* Subscribe to the necessay topics */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, vel_cb);
    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/offboardholy/target_attitude", 10, attitude_target_cb);
    ros::Subscriber sls_state_sub = nh.subscribe<offboardholy::PTStates>("/offboardholy/sls_state", 10, sls_state_cb);

    /* Publish the attitude and local position of the drone */
    ros::Publisher attitude_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    /* Service clients to arm/disarm and set mode of the drone */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    /* Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot. This loop should exit as soon as a heartbeat message is received. */
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /* Initialising the position to some default values */
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    /* Initialising the attitude to some default values */
    // mavros_msgs::AttitudeTarget attitude;
    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map";
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 0;
    attitude.thrust = 0.2;

    /* Send a few setpoints before starting and entering the Offboard mode, otherwise the mode switch will be rejected. */
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        // attitude.header.stamp = ros::Time::now();
        // attitude_setpoint_pub.publish(attitude);
        ros::spinOnce();
        rate.sleep();
    }

    /* Set the custom modes for OFFBOARD and AUTO LAND*/
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";

    ros::Time last_request = ros::Time::now();

    double distance = 0;
    int stage = 0;

    /* Main loop - Runs until ROS is running or any node fails */
    while (ros::ok())
    {
        double dv[10] = {};
        double controller_output[3] = {};
        // double Kv12[12] = {2.2361,    3.1623, 3.1623,   3.0777,    8.4827,    8.4827,  0,    9.7962,    9.7962,  0,    5.4399,    5.4399};
        double Kv12[12] = {2.2361, 3.1623, 3.1623, 3.0777, 8.4827, 8.4827, 0, 18.7962, 18.7962, 0, 17.4399, 17.4399};
        // double Kv12[12] = {3.0777,    5,  5,   2.2361,   6,    6,  0,     4,     4,  0,    3.1623,    3.1623};
        double Param[4] = {1.4, 0.08, 0.75, 9.8};
        double Setpoint[3] = {0, 0, -0.3};

        for (int i = 0; i < 10; i++)
        {
            dv[i] = PTState.PT_states[i];
            // ROS_INFO_STREAM( "dv[i]: "<< i << " : " << dv[i] << "\n");
        }

        /* Switch cases to control the drone autopilot */
        switch (stage)
        {
        case 0: // takeoff
            /* If the drone is not offboard and the lat request was made 5s ago: */
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                /* Try to set the drone to offboard */
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else
            {
                /* If the drone is not armed and the last request was made 5s ago then: */
                if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    /* Try to arm the drone */
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            /* Publish the initial postion setpoint */
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);

            /* Calculate the distance between the current local position and intial position */
            distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x), 2) + std::pow((current_local_pos.pose.position.y - pose.pose.position.y), 2) + std::pow((current_local_pos.pose.position.z - pose.pose.position.z), 2);

            /* Check if the drone has been airborne for more than 10s and the difference in distance is less than 0.2 then: */
            if (ros::Time::now() - last_request > ros::Duration(10.0) && distance < 0.2)
            {
                /* Move to the next stage */
                stage += 1;
                last_request = ros::Time::now();
                ROS_INFO("Takeoff finished and switch to position setpoint control mode");
            }
            break;

        case 1: // setpoint position control
            attitude.header.stamp = ros::Time::now();
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude_setpoint_pub.publish(attitude);

            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]), 2) + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])), 2) + std::pow((current_local_pos.pose.position.z - (-Setpoint[2] + 0.95)), 2);
            // ROS_INFO_STREAM("Distance: " << distance);
            if (ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2)
            {
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 1");
                last_request = ros::Time::now();
            }
            break;

        case 2: // setpoint position control
            Setpoint[0] = 1;
            Setpoint[1] = 0.5;
            Setpoint[2] = -0.6;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);

            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]), 2) + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])), 2) + std::pow((current_local_pos.pose.position.z - (-Setpoint[2] + 0.95)), 2);
            // ROS_INFO_STREAM(" X: " << current_local_pos.pose.position.x << " Y: " << current_local_pos.pose.position.y << " Z: " << current_local_pos.pose.position.z);
            ROS_INFO_STREAM("Distance: " << distance);
            if (ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2)
            {
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 2");
                last_request = ros::Time::now();
            }
            break;

        case 3: // setpoint position control
            Setpoint[0] = -1;
            Setpoint[1] = 0;
            Setpoint[2] = -0.3;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);

            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]), 2) + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])), 2) + std::pow((current_local_pos.pose.position.z - (-Setpoint[2] + 0.95)), 2);
            ROS_INFO_STREAM("Distance: " << distance);
            if (ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2)
            {
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 3");
                last_request = ros::Time::now();
            }
            break;

        case 4: // setpoint position control
            Setpoint[0] = 0;
            Setpoint[1] = 0;
            Setpoint[2] = -0.3;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);

            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]), 2) + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])), 2) + std::pow((current_local_pos.pose.position.z - (-Setpoint[2] + 0.95)), 2);
            ROS_INFO_STREAM("Distance: " << distance);
            if (ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2)
            {
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Trajectory Tracking");
                last_request = ros::Time::now();
            }
            break;

        case 5: // Trajectory tracking
            attitude.header.stamp = ros::Time::now();
            TracController(dv, Kv12, Param, ros::Time::now().toSec() - last_request.toSec(), controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude_setpoint_pub.publish(attitude);
            if (ros::Time::now() - last_request > ros::Duration(32.0))
            {
                stage += 1;
                ROS_INFO("Finish Trajactory Tracking and land");
                last_request = ros::Time::now();
            }
            break;

        case 6: // land
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0.5;
            local_pos_pub.publish(pose);
            distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x), 2) + std::pow((current_local_pos.pose.position.y - pose.pose.position.y), 2) + std::pow((current_local_pos.pose.position.z - pose.pose.position.z), 2);
            if (ros::Time::now() - last_request > ros::Duration(5.0))
            {
                // if(ros::Time::now() - last_request > ros::Duration(5.0) && distance < 0.2){

                // pose.header.stamp = ros::Time::now();
                // pose.header.frame_id = "map";
                // pose.pose.position.x = -0.5;
                // pose.pose.position.y = 0;
                // pose.pose.position.z = 0.5;
                // local_pos_pub.publish(pose);
                // distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2)
                // + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
                // + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);
                // if(ros::Time::now() - last_request > ros::Duration(10.0) && distance < 0.1){
                if (set_mode_client.call(land_mode) && land_mode.response.mode_sent)
                {
                    stage += 1;
                    ROS_INFO("Land finished");
                    // }
                }
            }
            break;

        default:
            if (set_mode_client.call(land_mode) && land_mode.response.mode_sent)
            {
                // arm_cmd.request.value = false;
                // arming_client.call(arm_cmd);
                // ROS_INFO("Land enabled");
            }
            break;
        }
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        // if((ros::Time::now() - last_request < ros::Duration(20.0))&& stage==0){
        //     pose.header.stamp = ros::Time::now();
        //     local_pos_pub.publish(pose);
        //     ROS_INFO("Position Control armed");
        // }else{
        //     stage += 1;
        // }

        // attitude.header.stamp = ros::Time::now();
        // attitude_setpoint_pub.publish(attitude);

        // ROS_INFO("Attitude Control armed");

        // distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2)
        // + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
        // + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);

        // if(distance < 0.1 && (ros::Time::now() - last_request > ros::Duration(50.0)) ){
        // 	if( set_mode_client.call(land_mode) &&
        //         land_mode.response.mode_sent){
        //         ROS_INFO("Land enabled");
        // 		break;
        //     }
        // }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude)
{
    attitude.header.stamp = ros::Time::now();
    double roll, pitch, yaw, thrust;
    thrust = sqrt(controller_output[0] * controller_output[0] + controller_output[1] * controller_output[1] + controller_output[2] * controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1] / thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);

    tf2::Quaternion attitude_target_q;
    attitude_target_q.setRPY(roll, pitch, yaw);
    attitude.orientation.x = attitude_target_q.getX();
    attitude.orientation.y = attitude_target_q.getY();
    attitude.orientation.z = attitude_target_q.getZ();
    attitude.orientation.w = attitude_target_q.getW();

    // attitude.thrust = (thrust-16.67122)/20 + 0.8168;
    attitude.thrust = (thrust - 14.504) / 8 + 0.78;

    //   ROS_INFO_STREAM("Force: " << controller_output[0]<< "   " << controller_output[1]<< "   " << controller_output[2] << " orientation " << roll << "  " << pitch);
}
