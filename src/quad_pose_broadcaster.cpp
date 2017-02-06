#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "quad_pose");
    ros::NodeHandle nh;

    ros::Publisher quadPose = nh.advertise<geometry_msgs::PoseStamped>("/quad_pose", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while(nh.ok()) {
        geometry_msgs::TransformStamped transform;
        try {
            transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch(tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PoseStamped origin;
        origin.pose.position.x = 0;
        origin.pose.position.y = 0;
        origin.pose.position.z = 0;
        origin.pose.orientation.x = 0;
        origin.pose.orientation.y = 0;
        origin.pose.orientation.z = 0;
        origin.pose.orientation.w = 1;

        geometry_msgs::PoseStamped result;

        tf2::doTransform(origin, result, transform);

        quadPose.publish(result);

        rate.sleep();
    }

    return 0;
}
