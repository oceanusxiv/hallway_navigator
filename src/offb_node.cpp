/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

float curr_dist;               // current distance
float prev_dist = 0;
float delta_dist;
float error;                    // our error that we use for thrust
double curr_time;
double prev_time = 0;
double delta_time;
float k_p = 0.3;                // our constant for error
float k_d = -0.3;                // the derivative constant
float thrust_const = 0.5;       // the constant of thrust
std_msgs::Float64 thrust_pwr;   // the published value. This is an object,
                                // not a c type, and should be treated as such

ros::Publisher thrust_pub;
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void cb_range(const sensor_msgs::Range::ConstPtr& rangePtr)
{
   curr_dist = rangePtr->range;
   error = 2 - curr_dist;                         // gets current error
   curr_time = ros::Time::now().toSec();          // gets current time
   delta_dist = curr_dist - prev_dist;            // computes delta dist
   delta_time = curr_time - prev_time;            // computes delta time
   prev_time  = curr_time;
   prev_dist = curr_dist;

   thrust_pwr.data = error * k_p + thrust_const + (delta_dist / delta_time) * k_d;
   if(thrust_pwr.data > 1)
       thrust_pwr.data = 1;
   else if(thrust_pwr.data < 0)
       thrust_pwr.data = 0;
   thrust_pub.publish(thrust_pwr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber range_sub  = nh.subscribe
            ("/terarangerone_corrected", 10, cb_range);
    thrust_pub = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time started = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
            started = ros::Time::now();
        } 
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                started = ros::Time::now();
            }
        }


        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
