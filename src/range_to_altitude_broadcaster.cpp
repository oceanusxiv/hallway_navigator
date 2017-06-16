#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/impl/utils.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Imu> MySyncPolicy;

geometry_msgs::TransformStamped range_transform;

void sensor_callback(const sensor_msgs::RangeConstPtr& range, const sensor_msgs::ImuConstPtr& imu){
    static tf2_ros::TransformBroadcaster br;

    double roll, pitch, yaw;
    tf2::impl::getEulerYPR(
            tf2::impl::toQuaternion(imu->orientation), yaw, pitch, roll);

    geometry_msgs::TransformStamped footprintTransform;
    footprintTransform.header.stamp = ros::Time::now();
    footprintTransform.header.frame_id = "map";
    footprintTransform.child_frame_id = "map_3d";
    footprintTransform.transform.translation.x = 0;
    footprintTransform.transform.translation.y = 0;
    footprintTransform.transform.translation.z = range->range * cos(roll) * cos(pitch);
    footprintTransform.transform.rotation = range_transform.transform.rotation;

    br.sendTransform(footprintTransform);
}

void static_callback(const tf2_msgs::TFMessageConstPtr& transform_msg) {
    for (auto& transform : transform_msg->transforms) {
        if (!transform.header.frame_id.compare("base_link") && !transform.child_frame_id.compare("base_range")) {
            ROS_INFO("Found static TF!");
            range_transform = transform;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_footprint_broadcaster");

    ros::NodeHandle nh;

    ros::Subscriber static_range_transform = nh.subscribe<tf2_msgs::TFMessage>("tf_static", 10, static_callback);
    message_filters::Subscriber<sensor_msgs::Range> teraranger_sub(nh, "terarangerone", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/mavros/imu/data", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), teraranger_sub, imu_sub);
    sync.registerCallback(boost::bind(&sensor_callback, _1, _2));

    ros::spin();

    return 0;
}