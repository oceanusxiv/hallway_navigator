#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Range range;
sensor_msgs::Imu imu_msg;
ros::Publisher lidar_corrected_pub;


//This code plagiarizef 
double metered_to_actual(double pitch, double yaw, double meter) {
  return meter * cos(pitch) * cos(yaw);
}

void teraranger_cb(const sensor_msgs::Range::ConstPtr& msg) {
  range = *msg;
  tf::Quaternion bt;
  tf::quaternionMsgToTF(imu_msg.orientation, bt);
  double roll, pitch, yaw;
  tf::Matrix3x3(bt).getRPY(roll, pitch, yaw);
  sensor_msgs::Range corrected;
  corrected.header = range.header;
  corrected.min_range = range.min_range;
  corrected.max_range = range.max_range;
  corrected.range = metered_to_actual(pitch, roll, range.range);
  lidar_corrected_pub.publish(corrected);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_msg = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_correction");
  ros::NodeHandle nh;
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::Range>
  ("/terarangerone", 10, teraranger_cb);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
  ("/mavros/imu/data", 10, imu_cb);
  lidar_corrected_pub = nh.advertise<sensor_msgs::Range>
  ("/terarangerone_corrected", 10);
  ros::spin();
}
