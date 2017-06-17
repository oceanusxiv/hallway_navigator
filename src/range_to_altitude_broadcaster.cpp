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
std::string base_link, base_range, frame, child_frame;

void sensor_callback(const sensor_msgs::RangeConstPtr& range, const sensor_msgs::ImuConstPtr& imu){
    static tf2_ros::TransformBroadcaster br;

    double roll, pitch, yaw;
    tf2::impl::getEulerYPR(
            tf2::impl::toQuaternion(imu->orientation), yaw, pitch, roll);

    geometry_msgs::TransformStamped footprintTransform;
    footprintTransform.header.stamp = ros::Time::now();
    footprintTransform.header.frame_id = frame;
    footprintTransform.child_frame_id = child_frame;
    footprintTransform.transform.translation.x = 0;
    footprintTransform.transform.translation.y = 0;
    footprintTransform.transform.translation.z = range->range * cos(roll) * cos(pitch);
    footprintTransform.transform.rotation = range_transform.transform.rotation;

    br.sendTransform(footprintTransform);
}

void static_callback(const tf2_msgs::TFMessageConstPtr& transform_msg) {
    for (auto& transform : transform_msg->transforms) {
        if (!transform.header.frame_id.compare(base_link) && !transform.child_frame_id.compare(base_range)) {
            ROS_INFO("Found static TF!");
            range_transform = transform;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_footprint_broadcaster");

    ros::NodeHandle nh;

    std::string range_topic, imu_topic;
    nh.param("range_topic",range_topic, "terarangerone");
    nh.param("imu_topic", imu_topic, "/mavros/imu/data");
    nh.param("base_link", base_link, "base_link");
    nh.param("base_range", base_range, "base_range");
    nh.param("frame", frame, "map");
    nh.param("child_frame", child_frame, "map_3d");

    ros::Subscriber static_range_transform = nh.subscribe<tf2_msgs::TFMessage>("tf_static", 10, static_callback);
    message_filters::Subscriber<sensor_msgs::Range> teraranger_sub(nh, range_topic, 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imu_topic, 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), teraranger_sub, imu_sub);
    sync.registerCallback(boost::bind(&sensor_callback, _1, _2));

    ros::spin();

    return 0;
}