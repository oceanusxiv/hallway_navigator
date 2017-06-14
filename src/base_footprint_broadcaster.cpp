#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/impl/utils.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

void callback(const sensor_msgs::RangeConstPtr& range, const sensor_msgs::ImuConstPtr& imu){
    static tf2_ros::TransformBroadcaster br;

    double roll, pitch, yaw;
    tf2::impl::getEulerYPR(
            tf2::impl::toQuaternion(imu->orientation), yaw, pitch, roll);

    geometry_msgs::TransformStamped footprintTransform;
    footprintTransform.header.stamp = ros::Time::now();
    footprintTransform.header.frame_id = "map";
    footprintTransform.child_frame_id = "map_2d";
    footprintTransform.transform.translation.x = 0.0;
    footprintTransform.transform.translation.y = 0.0;
    footprintTransform.transform.translation.z = range->range * cos(roll) * cos(pitch);
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

    message_filters::Subscriber<sensor_msgs::Range> teraranger_sub(nh, "terarangerone", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/mavros/imu/data", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Range, sensor_msgs::Imu> sync(teraranger_sub, imu_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}