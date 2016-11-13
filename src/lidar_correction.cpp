#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

float range_val;
geometry_msgs::Quaternion quat_orient;
ros::Publisher pub;

void cbRange(const sensor_msgs::Range::ConstPtr& rangePtr)
{
  range_val = rangePtr->range;
  sensor_msgs::Range range_info;

  tf::Quaternion tf_qt = tf::Quaternion(quat_orient.x, quat_orient.y, quat_orient.z, quat_orient.w);
  tf::Matrix3x3 m(tf_qt);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  range_info.range = range_val * cos(roll) * cos(pitch);
  range_info.radiation_type = rangePtr->radiation_type;
  range_info.field_of_view = rangePtr->field_of_view;
  range_info.min_range = rangePtr->min_range;
  range_info.max_range = rangePtr->max_range;

  pub.publish(range_info);
}

void cbImu(const sensor_msgs::Imu::ConstPtr& quatPtr)
{
  quat_orient = quatPtr->orientation;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "lidar_correction");
  ros::NodeHandle n;

  ros::Subscriber imusub = n.subscribe("/mavros/imu/data", 10, cbImu);
  ros::Subscriber rangesub = n.subscribe("/terarangerone", 10, cbRange);
  pub = n.advertise<sensor_msgs::Range>("/terarangerone_corrected", 10);

  ros::spin();

}
