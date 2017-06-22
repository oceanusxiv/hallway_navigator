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
#include <interactive_markers/interactive_marker_server.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <geometry_msgs/Point.h>

mavros_msgs::State current_state;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::string fixed_frame;
geometry_msgs::PoseStamped setpoint;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                                           << feedback->pose.position.x << ", " << feedback->pose.position.y
                                           << ", " << feedback->pose.position.z );
    setpoint.pose = feedback->pose;
}

visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale * 0.2;
    marker.scale.y = msg.scale * 0.2;
    marker.scale.z = msg.scale * 0.2;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void makeQuadrocopterMarker( const tf2::Vector3& position )
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = fixed_frame;
    int_marker.pose.position.x = position.x();
    int_marker.pose.position.y = position.y();
    int_marker.pose.position.z = position.z();
    int_marker.scale = 1;

    int_marker.name = "quadrocopter";
    int_marker.description = "Quadrocopter";

    makeBoxControl(int_marker);

    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
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

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("fixed_frame", fixed_frame, "map");
    std::cout << fixed_frame << std::endl;
    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
    makeQuadrocopterMarker( tf2::Vector3(0, 0, 0) );
    server->applyChanges();

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    setpoint.header.frame_id = fixed_frame;
    setpoint.header.stamp = ros::Time::now();

    while(ros::ok()){

        local_pos_pub.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
