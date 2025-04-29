#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path,Odometry
import math

# Global variables
current_pose = None
path = None

# Pure Pursuit parameters
lookahead_distance = 1.0  # Set the desired lookahead distance

def path_callback(msg):
    global path
    path = msg

def odom_callback(odom_msg):
    global current_pose
    # Extract the position from the odometry message
    # x = odom_msg.pose.pose.position.x
    # y = odom_msg.pose.pose.position.y
    # z = odom_msg.pose.pose.position.z
    current_pose = odom_msg.pose

    # # Extract the orientation from the odometry message
    # qx = odom_msg.pose.pose.orientation.x
    # qy = odom_msg.pose.pose.orientation.y
    # qz = odom_msg.pose.pose.orientation.z
    # qw = odom_msg.pose.pose.orientation.w


def follow_path():
    global path

    rospy.init_node('path_follower')
    rospy.Subscriber('/global/path', Path, path_callback)
    rospy.Subscriber('/my_odom', Odometry,odom_callback)
    cmd_pub = rospy.Publisher('/m4assembly/wheel_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    print(current_pose)
    goal_threshold = 0.1

    while not rospy.is_shutdown():
        if current_pose is None or path is None:
            continue

        # Get the current position of the robot
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y

        # Find the target point
        target_point = find_target_point(current_x, current_y)

        if target_point is None:
            rospy.loginfo("Path completed")
            cmd = Twist()  # Create a Twist message with zero velocities
            cmd_pub.publish(cmd)
            break

        # Calculate the distance and angle to the target
        distance = math.sqrt((target_point.x - current_x) ** 2 + (target_point.y - current_y) ** 2)
        angle = math.atan2(target_point.y - current_y, target_point.x - current_x)
        

        # Create a Twist message to control the robot
        cmd = Twist()
        cmd.linear.x = 1.0  # Constant speed of 1 m/s

        # Set the angular velocity to turn towards the target
        cmd.angular.z = 2 * math.sin(angle) / distance  # Pure Pursuit algorithm

        # Publish the Twist message
        cmd_pub.publish(cmd)

        rate.sleep()

def find_target_point(current_x, current_y):
    global path, lookahead_distance

    closest_distance = float('inf')
    closest_index = -1

    # Find the closest point on the path to the current position
    for i, pose in enumerate(path.poses):
        path_x = pose.pose.position.x
        path_y = pose.pose.position.y
        distance = math.sqrt((path_x - current_x) ** 2 + (path_y - current_y) ** 2)

        if distance < closest_distance:
            closest_distance = distance
            closest_index = i

    # Find the target point that is lookahead_distance away from the closest point
    for i in range(closest_index, len(path.poses)):
        path_x = path.poses[i].pose.position.x
        path_y = path.poses[i].pose.position.y
        distance = math.sqrt((path_x - current_x) ** 2 + (path_y - current_y) ** 2)

        if distance > lookahead_distance:
            return path.poses[i].pose.position

    return None

if __name__ == '__main__':
    try:
        follow_path()
    except rospy.ROSInterruptException:
        pass