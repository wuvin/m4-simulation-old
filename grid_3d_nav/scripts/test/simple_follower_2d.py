#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

# Parameters
WAYPOINT_THRESHOLD = 0.1  # Distance threshold to consider a waypoint reached
MAX_LINEAR_VEL = 0.5  # Maximum linear velocity
MAX_ANGULAR_VEL = 0.5  # Maximum angular velocity
END_GOAL_DISTANCE = 1.0  # Distance at which cmd_vel reduction starts
STOP_DURATION = 1.0  # Duration to stop and adjust heading in seconds

# Global variables
current_pose = None
path = None
state = "STOP"  # Initial state

# Smooth out the global plan
def smooth_path(global_plan):
    smoothed_path = []
    if len(global_plan.poses) < 3:
        return global_plan.poses

    # Add the start and end points unchanged
    smoothed_path.append(global_plan.poses[0])
    for i in range(1, len(global_plan.poses) - 1):
        smoothed_pose = PoseStamped()
        smoothed_pose.pose.position.x = (global_plan.poses[i - 1].pose.position.x + global_plan.poses[i + 1].pose.position.x) / 2
        smoothed_pose.pose.position.y = (global_plan.poses[i - 1].pose.position.y + global_plan.poses[i + 1].pose.position.y) / 2
        smoothed_path.append(smoothed_pose)
    smoothed_path.append(global_plan.poses[-1])

    #print(smoothed_path)
    return smoothed_path

# Calculate the desired cmd_vel
def calculate_cmd_vel():
    global current_pose, path

    if current_pose is None or path is None:
        return 0.0, 0.0

    # Get the current goal pose
    goal_pose = (path[0].pose.position.x, path[0].pose.position.y)

    # Calculate distance and angle to the goal
    dx = goal_pose[0] - current_pose.position.x
    dy = goal_pose[1] - current_pose.position.y
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.atan2(dy, dx)

    # Gradually reduce cmd_vel as the TurtleBot approaches the end goal
    if distance < END_GOAL_DISTANCE:
        linear_vel = MAX_LINEAR_VEL * (distance / END_GOAL_DISTANCE)
    else:
        linear_vel = MAX_LINEAR_VEL

    # Check the heading difference between current pose and goal pose
    _, _, current_yaw = cal_euler_from_quaternion(current_pose.orientation)
    yaw_to_goal = math.atan2(dy, dx)
    heading_diff = yaw_to_goal - current_yaw

    # Provide linear velocity only if the heading is consistent, otherwise angular velocity
    if abs(heading_diff) <= MAX_ANGULAR_VEL / 2:
        angular_vel = 0.0
    else:
        # Adjust angular velocity based on the heading difference
        angular_vel = heading_diff  # Use heading difference directly as angular velocity

        # Limit the angular velocity within the maximum range
        if angular_vel > MAX_ANGULAR_VEL:
            angular_vel = MAX_ANGULAR_VEL
        elif angular_vel < -MAX_ANGULAR_VEL:
            angular_vel = -MAX_ANGULAR_VEL

        #angular_vel = MAX_ANGULAR_VEL * (heading_diff / math.pi)
        linear_vel = 0.0

    return linear_vel, angular_vel

# Stop and adjust heading
def stop_and_adjust_heading():
    global state
    state = "STOP"
    rospy.sleep(STOP_DURATION)  # Stop for a fixed duration
    state = "ADJUST_HEADING"

# Convert quaternion to Euler angles
def cal_euler_from_quaternion(quaternion):
    return euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

# Callback for the path topic
def path_callback(msg):
    global path, state
    path = smooth_path(msg)
    #path=msg
    state = "FOLLOW_PATH"

# Callback for the current pose

def odom_callback(odom_msg):
    global current_pose
    # Extract the position from the odometry message
    current_pose = odom_msg.pose.pose
    #print(self.start)
# Main loop
def path_follower():
    rospy.init_node('path_follower')

    # Subscribe to the path and pose topics
    rospy.Subscriber('/global/path', Path, path_callback)
    rospy.Subscriber('/my_odom', Odometry, odom_callback)

    # Publish the cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/m4assembly/wheel_velocity_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        global path, state
        if path is not None and state != "FOLLOW_PATH":
            state = "FOLLOW_PATH"

        elif state == "FOLLOW_PATH":
            # Calculate cmd_vel
            linear_vel, angular_vel = calculate_cmd_vel()

            print(linear_vel,angular_vel)

            # Create Twist message with cmd_vel values
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_vel
            cmd_vel_msg.angular.z = angular_vel

            # Publish cmd_vel
            cmd_vel_pub.publish(cmd_vel_msg)

            # Check if the current waypoint is reached
            dx = path[0].pose.position.x - current_pose.position.x
            dy = path[0].pose.position.y - current_pose.position.y
            distance_to_goal = math.sqrt(dx ** 2 + dy ** 2)

            if distance_to_goal < WAYPOINT_THRESHOLD:
                path.pop(0)  # Move to the next waypoint

            if len(path) == 0:
                stop_and_adjust_heading()

        elif state == "STOP":
            # Create Twist message with zero cmd_vel values
            cmd_vel_msg = Twist()

            # Publish cmd_vel to stop the robot
            cmd_vel_pub.publish(cmd_vel_msg)

        elif state == "ADJUST_HEADING":
            # Calculate the angle to the goal
            dx = path[-1].pose.position.x - current_pose.position.x
            dy = path[-1].pose.position.y - current_pose.position.y
            angle = math.atan2(dy, dx)

            # Calculate the required angular velocity to adjust heading
            _, _, current_yaw = cal_euler_from_quaternion(current_pose.orientation)
            angular_vel = MAX_ANGULAR_VEL * (angle - current_yaw)

            # Create Twist message with zero linear velocity and required angular velocity
            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = angular_vel

            # Publish cmd_vel to adjust heading
            cmd_vel_pub.publish(cmd_vel_msg)

            # Check if the heading is aligned with the goal within a threshold
            if abs(angular_vel) < 0.01:
                state = "FOLLOW_PATH"

        rate.sleep()

if __name__ == '__main__':
    try:
        path_follower()
    except rospy.ROSInterruptException:
        pass