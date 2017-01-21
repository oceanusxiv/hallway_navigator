#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/impl/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>

void teraranger_cb(const sensor_msgs::Range::ConstPtr& msg){
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("base_stabilized", "base_range",
                                                    ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    double roll, pitch, yaw;
    tf2::impl::getEulerYPR(
            tf2::impl::toQuaternion(transformStamped.transform.rotation), yaw, pitch, roll);

    geometry_msgs::TransformStamped footprintTransform;
    footprintTransform.header.stamp = ros::Time::now();
    footprintTransform.header.frame_id = "base_footprint";
    footprintTransform.child_frame_id = "base_stabilized";
    footprintTransform.transform.translation.x = 0.0;
    footprintTransform.transform.translation.y = 0.0;
    footprintTransform.transform.translation.z = transformStamped.transform.translation.z +
            msg->range * cos(roll) * cos(pitch);
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    footprintTransform.transform.rotation.x = q.x();
    footprintTransform.transform.rotation.y = q.y();
    footprintTransform.transform.rotation.z = q.z();
    footprintTransform.transform.rotation.w = q.w();

    br.sendTransform(footprintTransform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_footprint_broadcaster");

    ros::NodeHandle nh;

    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::Range>
            ("/terarangerone", 10, teraranger_cb);

    ros::spin();

    return 0;
}