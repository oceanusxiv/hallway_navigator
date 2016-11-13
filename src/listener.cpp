#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

void statee_cb(const mavros_msgs::State::ConstPtr& msg) {
  
}

int main(int argc, char **argv) {
  ros:Rate = rate(20.0);

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber state_sub = nh.subscribe<std_msgs::State>
            ("mavros/state", 10, state_cb);
  ros::spin();
}
