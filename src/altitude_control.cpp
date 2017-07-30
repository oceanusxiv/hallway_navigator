/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;
sensor_msgs::Range lidar_range;
ros::Publisher throttle_pub;
static const double desired = 2.0;
static const double hover = 0.606;
static const double Kp = 0.01;
static const double Kd = -0.01;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void range_cb(const sensor_msgs::Range::ConstPtr& msg){
    static ros::Time last_time(0.0);
    static double last_range = 0.0;
    double step = (ros::Time::now() - last_time).toSec();
    last_time = ros::Time::now();
    lidar_range = *msg;
    double error = desired - lidar_range.range;
    double errord = (lidar_range.range - last_range)/step;
    std_msgs::Float64 throttle;
    throttle.data = std::max(std::min(error*Kp + errord*Kd + hover, 1.0), 0.0);
    last_range = lidar_range.range;
    throttle_pub.publish(throttle);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "altitude_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>
            ("/terarangerone_corrected", 10, range_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    throttle_pub = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);
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

    //send a few setpoints before starting
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

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
            started = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                started = ros::Time::now();
            }
        }



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
