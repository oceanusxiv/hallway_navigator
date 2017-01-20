#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
import tf2_ros
import tf
import geometry_msgs.msg

def callback_footprint(msg):
    br = tf2_ros.TransformBroadcaster()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = tfBuffer.lookup_transform("base_stabilized", "base_range", rospy.Time(), rospy.Duration(1.0))
    rospy.loginfo("%f"%trans.transform.translation.z)

    # t = geometry_msgs.msg.TransformStamped()
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "base_stabilized"
    # t.child_frame_id = "base_footprint"
    # t.transform.translation.x = 0.0
    # t.transform.translation.y = 0.0
    # t.transform.translation.z = msg.range
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

    # br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('base_footprint_broadcaster')
    rospy.Subscriber("/mavros/imu/data", Imu, callback_footprint)
    rospy.spin()