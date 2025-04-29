#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from grid_3d_nav.msg import PoseStampedwithflag,GlobalPath

def global_callback(msg):
    # Callback function for the second subscribed topic
    publish_with_nav_type(msg, "/m4nav/full_path")

def ground_callback(msg):
    # Callback function for the first subscribed topic
    publish_with_nav_type(msg, "/m4nav/ground_path")

def aerial_callback(msg):
    # Callback function for the second subscribed topic
    publish_with_nav_type(msg, "/m4nav/flight_waypoints")

def publish_with_nav_type(msg, new_topic_name):
    # Create a new message with the same data
    new_msg = Path()
    new_msg.header = msg.header

    for pose_stamped_flag in msg.poses:
        new_pose_stamped = PoseStamped()
        new_pose_stamped.header = pose_stamped_flag.header
        new_pose_stamped.pose = pose_stamped_flag.pose
        new_msg.poses.append(new_pose_stamped)

    # Publish the new message on the new topic
    pub = rospy.Publisher(new_topic_name, Path, queue_size=10)
    pub.publish(new_msg)

def main():
    rospy.init_node('nav_path_republisher', anonymous=True)
    rospy.Subscriber('/global/path', GlobalPath, global_callback)
    rospy.Subscriber('/ground/path', GlobalPath, ground_callback)
    rospy.Subscriber('/aerial/path', GlobalPath, aerial_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
