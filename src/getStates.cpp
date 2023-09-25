#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
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

#include <StabController.h>
#include <TracController.h>
#include <rtwtypes.h>
#include <cstddef>
#include <cstdlib>
# include <iostream>

// void gazebo_state_cb(const gazebo_msgs::LinkStates::ConstPtr& msg);
void PT_state_pub(ros::Publisher &sls_state_pub);
void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

geometry_msgs::PoseStamped current_local_pos;

geometry_msgs::TwistStamped load_vel;

geometry_msgs::Pose quadpose;
geometry_msgs::Pose loadpose;
geometry_msgs::Twist quadtwist;
geometry_msgs::Twist loadtwist;

offboardholy::PTStates PTState;

template<typename T>
T saturate(T val, T min, T max) {
    return std::min(std::max(val, min), max);
}

template<typename T>
T sigmoidf(T x) {
    return x/(1+std::abs(x));
}

struct PendulumAngles {
    double alpha, beta; // roll(alpha) pitch(beta) yaw
}penangle,penangle2;
PendulumAngles ToPenAngles(double Lx,double Ly,double Lz);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::TwistStamped current_local_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_local_vel = *msg;
}

struct sls_state {
    double x, y, z, alpha, beta, vx, vy, vz, gamma_alpha, gamma_beta;
}sls_state1;


geometry_msgs::PoseStamped load_pose, load_pose0;
double diff_time; 
void loadpose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    load_pose.header.frame_id = "map";
    
    load_pose.header.stamp = ros::Time::now();
    
    load_pose.pose.position.x = msg->transform.translation.x;
    load_pose.pose.position.y = msg->transform.translation.y;
    load_pose.pose.position.z = msg->transform.translation.z;
    load_pose.pose.orientation = msg->transform.rotation;
    
}

void timerCallback(const ros::TimerEvent&){
    load_vel.header.frame_id = "map";
    // diff_time = ros::Time::now().toSec() - load_pose.header.stamp.toSec();
    diff_time = 0.05;
    load_vel.header.stamp = ros::Time::now();
    load_vel.twist.linear.x = (load_pose.pose.position.x - load_pose0.pose.position.x)/diff_time;
    load_vel.twist.linear.y = (load_pose.pose.position.y - load_pose0.pose.position.y)/diff_time;
    load_vel.twist.linear.z = (load_pose.pose.position.z - load_pose0.pose.position.z)/diff_time;

    load_pose0.pose.position.x = load_pose.pose.position.x;
    load_pose0.pose.position.y = load_pose.pose.position.y;
    load_pose0.pose.position.z = load_pose.pose.position.z;
    
    // ROS_INFO_STREAM("Load Velocity: " << load_vel.twist.linear.x << "  " << load_vel.twist.linear.y << "  " << load_vel.twist.linear.z);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
  ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",10,vel_cb);
//   ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("gazebo/link_states", 10, gazebo_state_cb);
  ros::Subscriber load_vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/Load4Ball/Load4Ball",10, loadpose_cb);

  ros::Publisher sls_state_pub = nh.advertise<offboardholy::PTStates>("/offboardholy/sls_state", 10);
  ros::Publisher target_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/offboardholy/target_attitude", 10);


  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(50.0);

  mavros_msgs::AttitudeTarget attitude;
  attitude.header.stamp = ros::Time::now();
  attitude.header.frame_id = "map"; 
  attitude.orientation.x = 0;
  attitude.orientation.y = 0;
  attitude.orientation.z = 0;
  attitude.orientation.w = 0;
  attitude.thrust = 0.2;

    ros::Time last_request = ros::Time::now();

    double distance = 0;
    double t0 = 0;
    int stage = 0;

    while(ros::ok()){   
        PT_state_pub(sls_state_pub);


        double dv[10] = {};
        double controller_output[3] = {};
        double Kv12[12] = {2.2361,    3.1623, 3.1623,   3.0777,    8.4827,    8.4827,  0,    9.7962,    9.7962,  0,    5.4399,    5.4399};
        double Param[4] = {1.4, 0.08, 0.7, 9.8};
        double Setpoint[3] = {0, 0, -0.3};
        for (int i=0;i<10; i++){
          dv[i] = PTState.PT_states[i];
          // ROS_INFO_STREAM( "dv[i]: "<< i << " : " << dv[i] << "\n");
        }
        StabController(dv, Kv12, Param, Setpoint, controller_output);
        force_attitude_convert(controller_output, attitude);
        attitude.header.stamp = ros::Time::now();
        target_attitude_pub.publish(attitude);
        
        if( current_state.mode != "OFFBOARD"){
          t0 = ros::Time::now().toSec();
          // ROS_INFO_STREAM(t0);
        }
        
        // TracController(dv, Kv12, Param,  ros::Time::now().toSec()-t0, controller_output);
        

        // ROS_INFO_STREAM("gazebo position: " << quadpose.position.x  << "  " << quadpose.position.y << "  " << quadpose.position.z );
        // ROS_INFO_STREAM("px4 position: " << current_local_pos.pose.position.x << "  " << current_local_pos.pose.position.y << "  " << current_local_pos.pose.position.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void PT_state_pub(ros::Publisher &sls_state_pub){
    PTState.header.stamp = ros::Time::now();
    // PTState.PT_states[0] = sls_state1.x;
    // PTState.PT_states[1] = sls_state1.y;
    // PTState.PT_states[2] = sls_state1.z;
    PTState.PT_states[0] = current_local_pos.pose.position.x;
    PTState.PT_states[1] = -current_local_pos.pose.position.y;
    PTState.PT_states[2] = -current_local_pos.pose.position.z;
    PTState.PT_states[3] = sls_state1.alpha;
    PTState.PT_states[4] = sls_state1.beta;
    PTState.PT_states[5] = current_local_vel.twist.linear.x;
    PTState.PT_states[6] = -current_local_vel.twist.linear.y;
    PTState.PT_states[7] = -current_local_vel.twist.linear.z;
    // PTState.PT_states[5] = sls_state1.vx;
    // PTState.PT_states[6] = sls_state1.vy;
    // PTState.PT_states[7] = sls_state1.vz;
    PTState.PT_states[8] = sls_state1.gamma_alpha;
    PTState.PT_states[9] = sls_state1.gamma_beta;
    // PTState.PT_states[8] = 0;
    // PTState.PT_states[9] = 0;
    sls_state_pub.publish(PTState);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_local_pos = *msg;

    quadpose.position.x = current_local_pos.pose.position.x;
    quadpose.position.y = current_local_pos.pose.position.y;
    quadpose.position.z = current_local_pos.pose.position.z;

    loadpose.position.x = load_pose.pose.position.x;
    loadpose.position.y = load_pose.pose.position.y;
    loadpose.position.z = load_pose.pose.position.z;

    quadtwist.linear.x = current_local_vel.twist.linear.x;
    quadtwist.linear.y = current_local_vel.twist.linear.y;
    quadtwist.linear.z = current_local_vel.twist.linear.z;

    loadtwist.linear.x = load_vel.twist.linear.x;
    loadtwist.linear.y = load_vel.twist.linear.y;
    loadtwist.linear.z = load_vel.twist.linear.z;

    double Lx = (loadpose.position.x) - (quadpose.position.x) ;
    double Ly = (-loadpose.position.y) - (-quadpose.position.y) ;
    double Lz = (-loadpose.position.z) - (-quadpose.position.z) ;
    penangle = ToPenAngles( Lx, Ly, -Lz ); // in the paper the definition of n3 are opposite to the Z axis of gazebo
    sls_state1.alpha = penangle.alpha;
    sls_state1.beta = penangle.beta;

    double L = 1;
    double g_alpha, g_beta;
    g_beta = ((loadtwist.linear.x) - (quadtwist.linear.x))/(L*std::cos(sls_state1.beta));
    g_alpha = ((-loadtwist.linear.y) - (-quadtwist.linear.y) - std::sin(sls_state1.alpha)*std::sin(sls_state1.beta)*g_beta*L)/(-std::cos(sls_state1.alpha)*std::cos(sls_state1.beta)*L);

    sls_state1.gamma_alpha = g_alpha;
    sls_state1.gamma_beta = g_beta;
}


PendulumAngles ToPenAngles(double Lx,double Ly,double Lz) { //x=base.x
    PendulumAngles angles;
    // double L_cal = sqrt(Lx*Lx + Ly*Ly + Lz*Lz);
    // ROS_INFO_STREAM("Pendulum Length: " << L_cal);
    double L = 1;

    // ROS_INFO_STREAM(L);

    // beta (y-axis rotation)
    double sinbeta = Lx/L;
    double cosbeta = Lz/(L*std::cos(angles.alpha));
    angles.beta = std::asin(sinbeta);
    // ROS_INFO_STREAM("beta: " << angles.beta << "\n");
    // alpha (x-axis rotation)

    double cosa_cosb = Lz/L;
    double sina_cosb = Ly/-L;
    angles.alpha = std::asin(sina_cosb/std::cos(angles.beta));
    // ROS_INFO_STREAM("alpha: " << angles.alpha << "\n");

    return angles;
}

void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
  attitude.header.stamp = ros::Time::now();
  double roll,pitch,yaw, thrust;
  thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
  yaw = 0;
  roll = std::asin(controller_output[1]/thrust);
  pitch = std::atan2(controller_output[0], -controller_output[2]);

  tf2::Quaternion attitude_target_q;
  attitude_target_q.setRPY(roll, pitch, yaw);
  attitude.orientation.x = attitude_target_q.getX();
  attitude.orientation.y = attitude_target_q.getY();
  attitude.orientation.z = attitude_target_q.getZ();
  attitude.orientation.w = attitude_target_q.getW();

  // attitude.thrust = (thrust-16.67122)/20 + 0.8168;
  attitude.thrust = (thrust-14.504)/8 + 0.78;

//   ROS_INFO_STREAM("Force: " << controller_output[0]<< "   " << controller_output[1]<< "   " << controller_output[2] << " orientation " << roll << "  " << pitch << " Thrust: " << thrust << " X position: " << loadpose.position.x << " Y position: " << loadpose.position.y <<" Z position: " << loadpose.position.z);
}
















/////////////////////////////
// void gazebo_state_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
//     //ROS_INFO("I heard: [%s\n]", msg->name);
//     //current_state = *msg;
//     // cout<< msg->name[9]<< endl;

//     quadpose = msg->pose[2];
//     pendpose = msg->pose[9];
//     loadpose = msg->pose[10]; // 10: pose of load; 9: pose of pendulum
//     quadtwist = msg->twist[2];
//     loadtwist = msg->twist[10];

//     tf2::Quaternion quad_q(quadpose.orientation.x, quadpose.orientation.y, quadpose.orientation.z, quadpose.orientation.w);
//     tf2::Matrix3x3 quad_m(quad_q);
//     double quad_roll, quad_pitch, quad_yaw;
//     quad_m.getRPY(quad_roll, quad_pitch, quad_yaw);

//     sls_state1.x = quadpose.position.x;
//     sls_state1.y = -quadpose.position.y;
//     sls_state1.z = -quadpose.position.z;
//     sls_state1.vx = msg->twist[2].linear.x;
//     sls_state1.vy = -msg->twist[2].linear.y;
//     sls_state1.vz = -msg->twist[2].linear.z;


//     double Lx = (loadpose.position.x) - (quadpose.position.x) ;
//     double Ly = (-loadpose.position.y) - (-quadpose.position.y) ;
//     double Lz = (-loadpose.position.z) - (-quadpose.position.z) ;
//     penangle = ToPenAngles( Lx, Ly, -Lz ); // in the paper the definition of n3 are opposite to the Z axis of gazebo
//     sls_state1.alpha = penangle.alpha;
//     sls_state1.beta = penangle.beta;

//     double g_alpha, g_beta;
//     g_beta = ((loadtwist.linear.x) - (quadtwist.linear.x))/std::cos(sls_state1.beta);
//     g_alpha = ((-loadtwist.linear.y) - (-quadtwist.linear.y) - std::sin(sls_state1.alpha)*std::sin(sls_state1.beta)*g_beta)/(-std::cos(sls_state1.alpha)*std::cos(sls_state1.beta));

//     sls_state1.gamma_alpha = g_alpha;
//     sls_state1.gamma_beta = g_beta;
// }