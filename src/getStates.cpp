/*
About this file:
getStates.cpp code mainly deals with the communication with various topics through subscribing,
processing the data and then publishing the processed data to other topics.
*/

/* Includes necessary headers for ROS, geometry_msgs, mavros_msgs, and other custom headers. */
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

/* Includes the custom headers for the control functions. */
#include <StabController.h>
#include <TracController.h>
#include <rtwtypes.h>

/* Includes necessary headers for C++ standard library */
#include <cstddef>
#include <cstdlib>
#include <iostream>

// void gazebo_state_cb(const gazebo_msgs::LinkStates::ConstPtr& msg);

/* Declaring the functions defined below */
void PT_state_pub(ros::Publisher &sls_state_pub);
void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

/* Global variables for position, velocity and states */
geometry_msgs::PoseStamped current_local_pos;

geometry_msgs::TwistStamped load_vel;

geometry_msgs::Pose quadpose;
geometry_msgs::Pose loadpose;

geometry_msgs::Twist quadtwist;
geometry_msgs::Twist loadtwist;

offboardholy::PTStates PTState;

/* Template Function to limit the value between a certain range */
template <typename T>
T saturate(T val, T min, T max)
{
    return std::min(std::max(val, min), max);
}

/* Template Function to calculate the sigmoid function */
template <typename T>
T sigmoidf(T x)
{
    return x / (1 + std::abs(x));
}

/* Structure to store Pendulum angles -- alpha (roll) and beta (pitch) */
struct PendulumAngles
{
    double alpha, beta; // roll(alpha) pitch(beta) yaw
} penangle, penangle2;

/* Declaring the function defined below */
PendulumAngles ToPenAngles(double Lx, double Ly, double Lz);

mavros_msgs::State current_state;
/* Callback function to save the current state of the autopilot */
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::TwistStamped current_local_vel;
/* Callback function to save the current velocity of drone */
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_local_vel = *msg;
}

/* Structure to store the states of the slung load system */
struct sls_state
{
    double x, y, z, alpha, beta, vx, vy, vz, gamma_alpha, gamma_beta;
} sls_state1;

geometry_msgs::PoseStamped load_pose, load_pose0;
double diff_time;
/* Callback function to update the variable 'load_pose' with current position and orientation of the load from the msg param */
void loadpose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    load_pose.header.frame_id = "map";

    load_pose.header.stamp = ros::Time::now();

    load_pose.pose.position.x = msg->transform.translation.x;
    load_pose.pose.position.y = msg->transform.translation.y;
    load_pose.pose.position.z = msg->transform.translation.z;
    load_pose.pose.orientation = msg->transform.rotation;
}

/*
Callback function to calculate the velocity of the load based on change in postion of the load over time.
It then updates the variable 'load_pose0' with the current position of the load.
*/
void timerCallback(const ros::TimerEvent &)
{
    load_vel.header.frame_id = "map";
    // diff_time = ros::Time::now().toSec() - load_pose.header.stamp.toSec();
    diff_time = 0.05;
    load_vel.header.stamp = ros::Time::now();
    load_vel.twist.linear.x = (load_pose.pose.position.x - load_pose0.pose.position.x) / diff_time;
    load_vel.twist.linear.y = (load_pose.pose.position.y - load_pose0.pose.position.y) / diff_time;
    load_vel.twist.linear.z = (load_pose.pose.position.z - load_pose0.pose.position.z) / diff_time;

    load_pose0.pose.position.x = load_pose.pose.position.x;
    load_pose0.pose.position.y = load_pose.pose.position.y;
    load_pose0.pose.position.z = load_pose.pose.position.z;

    // ROS_INFO_STREAM("Load Velocity: " << load_vel.twist.linear.x << "  " << load_vel.twist.linear.y << "  " << load_vel.twist.linear.z);
}

int main(int argc, char **argv)
{
    /* Initialise ROS Node */
    ros::init(argc, argv, "listener");

    /* NodeHandle - Main access point for communications with ROS*/
    ros::NodeHandle nh;

    /* Subscribe to the topics for the state, postion and velocity of the drone and the load */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, vel_cb);
    // ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("gazebo/link_states", 10, gazebo_state_cb);
    /* Subscribe to vicon system for load position to help in tracking */
    ros::Subscriber load_vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/Load4Ball/Load4Ball", 10, loadpose_cb);

    /* Publish the states of the slung load system and the target attitude for the drone */
    ros::Publisher sls_state_pub = nh.advertise<offboardholy::PTStates>("/offboardholy/sls_state", 10);
    ros::Publisher target_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/offboardholy/target_attitude", 10);

    /* Service clients to arm/disarm and set mode of the drone */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    /* A timer to call function (timeCallback) at a regular intervals */
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    /* Initialising the drone's attitude (orientation and thrust) to some default values */
    mavros_msgs::AttitudeTarget attitude;
    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map";
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 0;
    attitude.thrust = 0.2;

    /* Storing the time of the last request */
    ros::Time last_request = ros::Time::now();

    /* Initialising some variables for drone controlling*/
    double distance = 0;
    double t0 = 0;
    int stage = 0;

    /* Main loop - Runs until ROS is running or any node fails */
    while (ros::ok())
    {
        /* Publish the states of the slung load system */
        PT_state_pub(sls_state_pub);

        /* Initialised some arrays for controller data */
        double dv[10] = {};
        double controller_output[3] = {};
        double Kv12[12] = {2.2361, 3.1623, 3.1623, 3.0777, 8.4827, 8.4827, 0, 9.7962, 9.7962, 0, 5.4399, 5.4399};
        double Param[4] = {1.4, 0.08, 0.7, 9.8};
        double Setpoint[3] = {0, 0, -0.3};

        for (int i = 0; i < 10; i++)
        {
            dv[i] = PTState.PT_states[i];
            // ROS_INFO_STREAM( "dv[i]: "<< i << " : " << dv[i] << "\n");
        }

        /* Calling Stabilisation controller with the following arguments */
        StabController(dv, Kv12, Param, Setpoint, controller_output);

        /* Converting the control output from the controller to attitude target for drone adn publishing it */
        force_attitude_convert(controller_output, attitude);
        attitude.header.stamp = ros::Time::now();
        target_attitude_pub.publish(attitude);

        /* If the current mode is not OFFBOARD -> Update the start time */
        if (current_state.mode != "OFFBOARD")
        {
            t0 = ros::Time::now().toSec();
            // ROS_INFO_STREAM(t0);
        }

        // TracController(dv, Kv12, Param,  ros::Time::now().toSec()-t0, controller_output);

        // ROS_INFO_STREAM("gazebo position: " << quadpose.position.x  << "  " << quadpose.position.y << "  " << quadpose.position.z );
        // ROS_INFO_STREAM("px4 position: " << current_local_pos.pose.position.x << "  " << current_local_pos.pose.position.y << "  " << current_local_pos.pose.position.z);

        /* spinOnce to handle all the callbacks */
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/*
Function to publish the states of the drone system to the ROS topics

It updates the PTState.PT_states array with the current position, velocity and angles of the drone system.
It then uses sls_state_pub publisher to publish the PTState message to the particular ROS topic.
*/
void PT_state_pub(ros::Publisher &sls_state_pub)
{
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

/*
Callback function to update the current state of drone and load based on the pose message received.

This function updates the current local position and velocity of the drone and the load, whenever a new pose message is received.
It also calculates the angles of the load (sls) based on the relative position of the drone and the load.
It then calculates the gamma_alpha and gamma_beta values based on the angles recieved from the ToPenAngles, velocities of both the drone and the load, the pendulum's length (L).
*/
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_local_pos = *msg;

    /* Update drone/quadrotor position */
    quadpose.position.x = current_local_pos.pose.position.x;
    quadpose.position.y = current_local_pos.pose.position.y;
    quadpose.position.z = current_local_pos.pose.position.z;

    /* Update load position */
    loadpose.position.x = load_pose.pose.position.x;
    loadpose.position.y = load_pose.pose.position.y;
    loadpose.position.z = load_pose.pose.position.z;

    /* Update drone/quadrotor velocity */
    quadtwist.linear.x = current_local_vel.twist.linear.x;
    quadtwist.linear.y = current_local_vel.twist.linear.y;
    quadtwist.linear.z = current_local_vel.twist.linear.z;

    /* Update load velocity */
    loadtwist.linear.x = load_vel.twist.linear.x;
    loadtwist.linear.y = load_vel.twist.linear.y;
    loadtwist.linear.z = load_vel.twist.linear.z;

    /* Calculate the relative position of load - x, y, z components of the load*/
    double Lx = (loadpose.position.x) - (quadpose.position.x);
    double Ly = (-loadpose.position.y) - (-quadpose.position.y);
    double Lz = (-loadpose.position.z) - (-quadpose.position.z);

    /* Getting the angles of the pendulum based on the relative position of the load */
    penangle = ToPenAngles(Lx, Ly, -Lz); // in the paper the definition of n3 are opposite to the Z axis of gazebo

    sls_state1.alpha = penangle.alpha;
    sls_state1.beta = penangle.beta;

    /* Calculate the gamma_alpha and gamma_beta values based on the angles recieved from the ToPenAngles, velocities of both the drone and the load, the pendulum's length (L). */
    double L = 1; // length of the pendulum
    double g_alpha, g_beta;

    g_beta = ((loadtwist.linear.x) - (quadtwist.linear.x)) / (L * std::cos(sls_state1.beta));
    g_alpha = ((-loadtwist.linear.y) - (-quadtwist.linear.y) - std::sin(sls_state1.alpha) * std::sin(sls_state1.beta) * g_beta * L) / (-std::cos(sls_state1.alpha) * std::cos(sls_state1.beta) * L);

    sls_state1.gamma_alpha = g_alpha;
    sls_state1.gamma_beta = g_beta;
}

/*
Function that calculates the angles of the pendulum based on the relative position of the load.

This function calculates the angles of the pendulum based on the relative position of the load and the length of the pendulum.
It then returns the structure 'angles' which contain the calculated angles (alpha and beta).
*/
PendulumAngles ToPenAngles(double Lx, double Ly, double Lz)
{ // x=base.x
    PendulumAngles angles;
    // double L_cal = sqrt(Lx*Lx + Ly*Ly + Lz*Lz);
    // ROS_INFO_STREAM("Pendulum Length: " << L_cal);
    double L = 1;

    // ROS_INFO_STREAM(L);

    // beta (y-axis rotation)
    double sinbeta = Lx / L;
    double cosbeta = Lz / (L * std::cos(angles.alpha));
    angles.beta = std::asin(sinbeta);
    // ROS_INFO_STREAM("beta: " << angles.beta << "\n");
    // alpha (x-axis rotation)

    double cosa_cosb = Lz / L;
    double sina_cosb = Ly / -L;
    angles.alpha = std::asin(sina_cosb / std::cos(angles.beta));
    // ROS_INFO_STREAM("alpha: " << angles.alpha << "\n");

    return angles;
}

/*
This function converts the force output from the controller to the attitude target/commands (roll, pitch, yaw and thrust) for the drone.

controller_output - contains the force output from the controller.
attitude - reference to the attitude target for the drone to be updated.
*/
void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude)
{
    attitude.header.stamp = ros::Time::now();

    double roll, pitch, yaw, thrust;

    thrust = std::sqrt(controller_output[0] * controller_output[0] + controller_output[1] * controller_output[1] + controller_output[2] * controller_output[2]);
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