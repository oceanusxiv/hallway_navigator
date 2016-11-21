#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import Range
import tf


def callback_footprint(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, msg.range),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), "base_stabilized", "base_footprint")

if __name__ == '__main__':
    rospy.init_node('base_footprint_broadcaster')
    rospy.Subscriber("/terarangerone_corrected", Range, callback_footprint)
    rospy.spin()