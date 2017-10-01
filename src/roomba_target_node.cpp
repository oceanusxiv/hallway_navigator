//TODO: CHANGE THIS FORMAT INTO A CLASS SO THAT IT IS MORE MODULAR

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include <sensors/sensors.hh>


//void buttonPressed(const gazebo::sensors::RaySensor &sens){}

// void random 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roomba_target_node");
    ros::NodeHandle nh_; 

    ros::Publisher pub = nh_.advertise<geometry_msgs::Twist>("/Robot2/cmd_vel", 1);

    geometry_msgs::Twist msg;
    
    // First begin by moving forward??? IDK IF GOOD IDEA
    msg.linear.x = 0.1;
    pub.publish(msg);

    ros::Rate rate(0.05);

    while (ros::ok()) {
        // TODO: RANDOMIZE

        ROS_INFO("ROBOT IS NOW TURNING!!");

        msg.linear.x = 0.0; // Stop temporarily

        msg.angular.z = 0.1745 * 5; // 50 degrees per second angular velocity
        pub.publish(msg);
        ros::Duration(1).sleep(); // Sleep 1 sec - SHOULD BE RANDOM
        msg.angular.z = 0.0; // Stop rotating
        pub.publish(msg);

        msg.linear.x = 0.1; // Resume
        pub.publish(msg);

        rate.sleep();
    }


    return 0;
}
