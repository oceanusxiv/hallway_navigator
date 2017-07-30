//
// Created by Eric Fang on 7/17/17.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_pose = *msg;
}

std::vector<geometry_msgs::Point> read_waypoints(std::string path) {
    std::string line;
    std::ifstream waypoint_file (path);
    std::vector<geometry_msgs::Point> waypoints;
    if (waypoint_file.is_open())
    {
        while ( std::getline (waypoint_file, line) )
        {
            std::vector<std::string> waypoint;
            boost::split(waypoint, line, boost::is_any_of(","));
            if (waypoint.size() != 3) {
                std::cout << "format incorrect!" << std::endl;
                return waypoints;
            }
            geometry_msgs::Point point;
            point.x = std::stod(waypoint[0]);
            point.y = std::stod(waypoint[1]);
            point.z = std::stod(waypoint[2]);
            ROS_INFO("%f, %f, %f", point.x, point.y, point.z);
            waypoints.push_back(point);
        }
        waypoint_file.close();
        return waypoints;
    }

    else {
        std::cout << "Unable to open file";
        return waypoints;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string waypoints_file;
    double epsilon;
    nh_private.param<std::string>("waypoints_file", waypoints_file, "./waypoints.txt");
    nh_private.param<double>("margin", epsilon, 0.1);

    std::vector<geometry_msgs::Point> waypoints = read_waypoints(waypoints_file);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
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
    pose.pose.position.z = 0;

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

    int progress_counter = 0;

    while(ros::ok()) {

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        double error_x = current_pose.pose.position.x - pose.pose.position.x;
        double error_y = current_pose.pose.position.y - pose.pose.position.y;
        double error_z = current_pose.pose.position.z - pose.pose.position.z;
        double error = std::sqrt(error_x*error_x + error_y*error_y + error_z*error_z);

        if(error < epsilon && progress_counter < waypoints.size()) {
            pose.pose.position = waypoints[progress_counter];
            progress_counter++;
        }
        else if(error < epsilon && progress_counter >= waypoints.size()) {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if( current_state.mode != "AUTO.LAND" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("landing");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}