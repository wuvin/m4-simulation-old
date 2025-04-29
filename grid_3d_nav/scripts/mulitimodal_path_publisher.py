#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path,Odometry
from grid_3d_nav.msg import PoseStampedwithflag,GlobalPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64,Int32

def odom_callback(odom_msg):
    global robot_x ,robot_y,robot_z
    # Extract the position from the odometry message
    robot_x = odom_msg.pose.pose.position.x
    robot_y = odom_msg.pose.pose.position.y
    robot_z = odom_msg.pose.pose.position.z

def split_path(big_path_msg):
    prev_flag = None
    segments = []
    current_segment = GlobalPath()
    current_segment.header = big_path_msg.header

    for poses_with_flags in big_path_msg.poses:
        pose = poses_with_flags
        current_flag = poses_with_flags.flag

        if prev_flag is not None and prev_flag != current_flag:
            rospy.loginfo("path published")
            publish_path_segment(current_segment)
            next_path = wait_for_goal_achievement(pose,current_segment)
            current_segment.poses.clear()

        current_segment.poses.append(pose)
        prev_flag = current_flag 
    if current_segment.poses:
        publish_path_segment(current_segment)  
    return 

def publish_path_segment(current_segment):
    if current_segment.poses[0].flag == 1 :
        change_mode(1)
        rospy.sleep(3)
        aerial_path_publisher.publish(current_segment)
    elif current_segment.poses[0].flag == 0 :
        change_mode(0)
        rospy.sleep(3)
        ground_path_publisher.publish(current_segment)
    return

def wait_for_goal_achievement(aerial_l_goal,current_segment,threshold=0.1):
    last_pose = current_segment.poses[-1]
    l_goal_x = last_pose.pose.position.x
    l_goal_y = last_pose.pose.position.y
    l_goal_z = last_pose.pose.position.z


    #TO KNOW IF FLIGHT GOAL IS ACHIVED AS LANDING POINT IS NOT DESCRIBED
    flight_goal_x = aerial_l_goal.pose.position.x
    flight_goal_y = aerial_l_goal.pose.position.y
    flight_goal_z = aerial_l_goal.pose.position.z

    while not rospy.is_shutdown():

        if current_segment.poses[0].flag == 0 :
            off_x = abs(l_goal_x) - abs(robot_x)
            off_y = abs(l_goal_y) - abs(robot_y)
            off_z = abs(l_goal_z) - abs(robot_z)

            if off_x <= threshold and off_y <= threshold and off_z <= threshold:
                return True  # Goal achieved, return True
        
        elif current_segment.poses[0].flag == 1 :
            off_x = abs(flight_goal_x) - abs(robot_x)
            off_y = abs(flight_goal_y) - abs(robot_y)
            off_z = abs(flight_goal_z) - abs(robot_z)

            if off_x <= threshold and off_y <= threshold and off_z <= threshold:
                return True  # Goal achieved, return True

        rospy.sleep(0.5)  # Wait for the robot to achieve the goal
        rospy.loginfo('the goal is not yet achived')

    return False

def change_mode(flag):
    # #Joint publisher for all 4 joints so that it can change shape
    pub1 = rospy.Publisher('/m4assembly/front_left_hip_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/m4assembly/front_right_hip_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/m4assembly/rear_left_hip_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/m4assembly/rear_right_hip_position_controller/command', Float64, queue_size=10)

    #for real robot transformation flag
    changeflag = Int32()
    changeflag.data = int(flag)
    switch_flag.publish(changeflag)

    if flag == 1:
        x = np.deg2rad(90)
        for i in range(1, 90):
            pub1.publish(Float64(np.deg2rad(i)))
            pub2.publish(Float64(np.deg2rad(i)))
            pub3.publish(Float64(np.deg2rad(i)))
            pub4.publish(Float64(np.deg2rad(i)))
            rospy.sleep(0.01)
    else :
        x=np.deg2rad(0)

        for i in range(1, 90):
            pub1.publish(Float64(np.deg2rad(90-i)))
            pub2.publish(Float64(np.deg2rad(90-i)))
            pub3.publish(Float64(np.deg2rad(90-i)))
            pub4.publish(Float64(np.deg2rad(90-i)))
            rospy.sleep(0.01)
    
    return rospy.loginfo('mode of the robot changed')

def nav_path_callback(big_path_msg):
    try:
        # Split the big path into segments
        split_path(big_path_msg)

    except Exception as e:
        rospy.logerr('Error occurred: {}'.format(str(e)))

if __name__ == '__main__':

    rospy.init_node('multimodal_path_publisher')
    ground_path_publisher = rospy.Publisher('/ground/path', GlobalPath, queue_size=10)
    aerial_path_publisher = rospy.Publisher('/aerial/path', GlobalPath, queue_size=10)
    switch_flag = rospy.Publisher('/m4nav/transform_state', Int32, queue_size=10)

    rospy.Subscriber('/my_odom', Odometry,odom_callback)
    rospy.Subscriber('/global/path', GlobalPath, nav_path_callback)

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_message('/global/path', GlobalPath, timeout=1.0)
            rospy.Subscriber('/global/path', GlobalPath, nav_path_callback)
            rospy.spin()

        except rospy.ROSException as e:
            rospy.loginfo('Waiting for big path message...')
            rospy.sleep(1.0)

        except Exception as e:
            rospy.logerr('Error occurred: {}'.format(str(e)))

    rospy.loginfo('Exiting the multimodal path publisher node.')
