#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from grid_3d_nav.msg import PoseStampedwithflag,GlobalPath
from geometry_msgs.msg import Twist, Point
from math import atan2, pi

x = 0.0
y = 0.0
theta = 0.0
path = []
path_completed = True

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newPath(path_msg):
    global path
    global path_completed
    path = path_msg.poses
    path_completed = False
    
def stopRobot():
	speed.linear.x = 0.0
	speed.angular.z = 0.0
	pub.publish(speed)

rospy.init_node("path_follower")

sub_odom = rospy.Subscriber("/my_odom", Odometry, newOdom)
sub_path = rospy.Subscriber("/ground/path", GlobalPath, newPath)
#for simulation
pub = rospy.Publisher("/m4assembly/wheel_velocity_controller/cmd_vel", Twist, queue_size=1)
#for onboard robot
#pub = rospy.Publisher("m4nav/cmd_vel", Twist, queue_size=1)

speed = Twist()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if not path:
        stopRobot()
        continue
    elif path_completed:
        stopRobot()
        rate.sleep()
        continue

    goal = path[0].pose.position
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x)

    # Calculate the angle difference while keeping it within -180° to 180° range
    angle_diff = angle_to_goal - theta
    if angle_diff > pi:
        angle_diff -= 2 * pi
    elif angle_diff < -pi:
        angle_diff += 2 * pi

    if abs(angle_diff) > 0.1:
        speed.linear.x = 0.0

        # Adjust the angular velocity direction based on the angle difference
        if angle_diff > 0:
            speed.angular.z = 0.8
        else:
            speed.angular.z = -0.8
    else:
        speed.linear.x = 0.9
        speed.angular.z = 0.0
        if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
            path = path[1:]  # Move to the next waypoint
        if not path :
            path_completed = True
    pub.publish(speed)
    rate.sleep()

# Stop the robot when the script is shut down
speed.linear.x = 0.0
speed.angular.z = 0.0
pub.publish(speed)



