#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/impl/utils.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>

geometry_msgs::TransformStamped range_transform;
sensor_msgs::Imu imu;
std::string base_link, base_range, frame, child_frame;

void range_callback(const sensor_msgs::RangeConstPtr& range_msg) {
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped footprintTransform;
    footprintTransform.header.stamp = ros::Time::now();
    footprintTransform.header.frame_id = frame;
    footprintTransform.child_frame_id = child_frame;
    footprintTransform.transform.translation.x = 0;
    footprintTransform.transform.translation.y = 0;
    auto rotation = geometry_msgs::Quaternion();
    rotation.w = 1;
    footprintTransform.transform.rotation = rotation;

    if (imu.orientation.x == 0 && imu.orientation.y == 0 && imu.orientation.z == 0 && imu.orientation.w == 0) {
        footprintTransform.transform.translation.z = range_msg->range;
    }
    else {
        Eigen::Vector3d range(0, 0, range_msg->range);
        Eigen::Vector3d base_to_range(range_transform.transform.translation.x,
                                      range_transform.transform.translation.y,
                                      range_transform.transform.translation.z);
        auto base_to_ground = base_to_range + range;
        Eigen::Quaterniond q;
        q.w() = imu.orientation.w;
        q.x() = imu.orientation.x;
        q.y() = imu.orientation.y;
        q.z() = imu.orientation.z;
        auto rotated_base_to_ground = q.toRotationMatrix() * base_to_ground;

        footprintTransform.transform.translation.z = rotated_base_to_ground.z();
    }

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

void imu_callback(sensor_msgs::Imu imu_msg) {
    imu = imu_msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_footprint_broadcaster");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string range_topic, imu_topic;
    nh_private.param<std::string>("~range_topic",range_topic, "terarangerone");
    nh_private.param<std::string>("~imu_topic", imu_topic, "/mavros/imu/data");
    nh_private.param<std::string>("~base_link", base_link, "base_link");
    nh_private.param<std::string>("~base_range", base_range, "base_range");
    nh_private.param<std::string>("~frame", frame, "map");
    nh_private.param<std::string>("~child_frame", child_frame, "map_3d");

    ros::Subscriber static_range_transform = nh.subscribe<tf2_msgs::TFMessage>("tf_static", 10, static_callback);
    ros::Subscriber teraranger_sub = nh.subscribe<sensor_msgs::Range>(range_topic, 10, range_callback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, imu_callback);

    ros::spin();

    return 0;
}