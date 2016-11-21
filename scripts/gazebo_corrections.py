#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Header


class GazeboCorrections(object):

    def __init__(self):
        rospy.init_node('gazebo_listener')
        self.pub_scan = rospy.Publisher('/rplidar_synced/scan', LaserScan, queue_size=10)
        self.pub_teraranger = rospy.Publisher('/terarangerone', Range, queue_size=10)
        rospy.Subscriber("/rplidar/scan", LaserScan, self.callback_scan)
        rospy.Subscriber("/teraranger", LaserScan, self.callback_teraranger)
        rospy.spin()


    def callback_scan(self, data):
        msg = data
        msg.header.frame_id = "laser_link"
        msg.header.stamp = rospy.get_rostime()
        self.pub_scan.publish(data)

    def callback_teraranger(self, data):
        msg = Range()
        msg.header = Header()
        msg.header.seq = data.header.seq
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.get_rostime()
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.0523598776
        msg.min_range = data.range_min
        msg.max_range = data.range_max
        msg.range = data.ranges[0]
        self.pub_teraranger.publish(msg)
    
if __name__ == '__main__':
    gc = GazeboCorrections()