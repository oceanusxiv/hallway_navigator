#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray


class GazeboCorrections(object):

    def __init__(self):
        rospy.init_node('gazebo_listener')
        self.pub_scan = rospy.Publisher('/rplidar_synced/scan', LaserScan, queue_size=10)
        self.pub_teraranger = rospy.Publisher('/terarangerone', Range, queue_size=10)
        self.pub_imu = rospy.Publisher('/mavros/imu/data_synced', Imu, queue_size=10)
        rospy.Subscriber("/rplidar/scan", LaserScan, self.callback_scan)
        rospy.Subscriber("/teraranger", LaserScan, self.callback_teraranger)
        rospy.Subscriber("/mavros/imu/data", Imu, self.callback_imu)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.callback_time)
        self.time_offset = rospy.Duration.from_sec(0)
        rospy.spin()

    def callback_scan(self, data):
        msg = LaserScan()
        msg.header = Header()
        msg.header.frame_id = "laser_link"
        msg.header.stamp = data.header.stamp
        msg.header.seq = data.header.seq
        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max
        msg.ranges = data.ranges
        msg.intensities = data.intensities
        self.pub_scan.publish(msg)

    def callback_teraranger(self, data):
        msg = Range()
        msg.header = Header()
        msg.header.seq = data.header.seq
        msg.header.frame_id = "base_link"
        msg.header.stamp = data.header.stamp
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.0523598776
        msg.min_range = data.range_min
        msg.max_range = data.range_max
        msg.range = data.ranges[0]
        self.pub_teraranger.publish(msg)

    def callback_time(self, data):
        self.time_offset = rospy.Duration.from_sec(float(data.status[5].values[5].value))

    def callback_imu(self, data):
        msg = data
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.get_rostime()
        self.pub_imu.publish(msg)
    
if __name__ == '__main__':
    gc = GazeboCorrections()