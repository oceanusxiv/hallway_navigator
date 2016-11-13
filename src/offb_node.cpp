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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber position_sub = nh.subscribe<sensor_msgs::Range>
            ("mavros/terarangerone_corrected", 10);

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

    ros::Publisher thrust_publisher = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);
    Float64 thrust;
    float thrustAdjustment = .1;
    float desiredPosition = 2;
    float z = position_sub.range;
    float offsetThrust = thrustAdjustment*(desiredPosition - z);
    float acceptableError = .2;
    float steadyThrust = .606;
    double lastTime = ros::Time:now().toSec();
    double deltaTime = 1;
    float lastZ = position_sub.range;
    float velocityAdjustment = -.003;
    double deltaPosVsTime;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
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
	// change the thrust until the error is nothing
	deltaTime = ros::Time::now().toSec() - lastTime;
	lastTime = ros::Time::now();
        deltaPosVsTime = (z-lastZ)/(deltaTime)*velocityAdjustment;
        lastZ = z;
        z = position_sub.range;
        OffsetThrust = thrustAdjustment*(desiredPosition - z);
	if((desiredPosition - z)>acceptableError)
	{
	    thrust = offsetThrust + steadyThrust + deltaPosVsTime;
	}
	else
	{
	    thrust = steadyThrust;
	}
        thrust_publisher.publish(thrust)
        

        if (current_state.armed && 
            ros::Time::now() - started > ros::Duration(25) && 
            ros::Time::now() - last_request > ros::Duration(5)) {
            arm_cmd.request.value = false;
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
            }  
            last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
