#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Create a new TransformStamped message
    odom_tf = tf2_ros.TransformStamped()

    # Fill in the header information
    odom_tf.header.stamp = rospy.Time.now()
    odom_tf.header.frame_id = parent_frame
    odom_tf.child_frame_id = child_frame

    # Fill in the transform information
    odom_tf.transform.translation.x = msg.pose.pose.position.x
    odom_tf.transform.translation.y = msg.pose.pose.position.y
    odom_tf.transform.translation.z = msg.pose.pose.position.z
    odom_tf.transform.rotation.x = msg.pose.pose.orientation.x
    odom_tf.transform.rotation.y = msg.pose.pose.orientation.y
    odom_tf.transform.rotation.z = msg.pose.pose.orientation.z
    odom_tf.transform.rotation.w = msg.pose.pose.orientation.w

    # Send the transform
    tf2_ros.TransformBroadcaster().sendTransform(odom_tf)

if __name__ == '__main__':
    rospy.init_node('odom2tf')
    topic = rospy.get_param('~odom_topic')
    parent_frame = rospy.get_param('~parent_frame')
    child_frame = rospy.get_param('~child_frame')

    try:
        while not rospy.is_shutdown():
            rospy.Subscriber(topic, Odometry, odom_callback)
            rospy.spin()

    except rospy.ROSInterruptException:
        quit()  