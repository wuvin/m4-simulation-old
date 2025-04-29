#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointClear, WaypointPush, CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State,Waypoint,CommandCode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


current_state = State()
current_pose = PoseStamped()
waypoint_mission_sent = False

def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def send_waypoint_mission(waypoints):
    # Clear existing mission
    rospy.wait_for_service('/mavros/mission/clear')
    clear_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
    clear_service()

    # Push new mission
    rospy.wait_for_service('/mavros/mission/push')
    push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
    push_service(0, waypoints)

def set_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode_service(0, mode)
        return response.mode_sent
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False

def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arm_service(True)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False

def disarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        disarm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = disarm_service(False)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False

def takeoff(altitude):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        response = takeoff_service(0, 0, 0, 0, altitude)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False

def land():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = land_service(0, 0, 0, 0, 0)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False

def path_callback(msg):
    global waypoint_mission_sent
    # Check if the path message is empty or not
    if not msg.poses:
        rospy.logwarn("Received an empty path message. Please publish waypoints.")
        return

    if not waypoint_mission_sent:
        rospy.loginfo("Received path, starting the flight sequence...")
        waypoint_mission_sent = True

        # Change mode to AUTO.MISSION
        if not set_mode('AUTO.MISSION'):
            rospy.logerr("Failed to set AUTO.MISSION mode!")
            return

        # Arm the vehicle
        if not arm():
            rospy.logerr("Failed to arm the vehicle!")
            return

        # Takeoff to a desired altitude
        target_altitude = 10.0  # Define the desired altitude (in meters)
        if not takeoff(target_altitude):
            rospy.logerr("Failed to takeoff!")
            return

        # Extract waypoints from the path message
        waypoints = []
        for pose in msg.poses:
            waypoint = create_waypoint(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            waypoints.append(waypoint)

        # Send the waypoint mission to ArduPilot
        send_waypoint_mission(waypoints)

    else:
        rospy.loginfo("Waypoint mission already sent. Ignoring subsequent path messages.")

def create_waypoint(x, y, z):
    waypoint = Waypoint()
    waypoint.frame = Waypoint.FRAME_LOCAL_NED
    waypoint.command = CommandCode.NAV_WAYPOINT
    waypoint.is_current = False
    waypoint.autocontinue = True
    waypoint.param1 = 0
    waypoint.param2 = 0
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = x
    waypoint.y_long = y
    waypoint.z_alt = z

    return waypoint

def main():
    rospy.init_node('flight_control_node')

    # Subscribe to the nav/path topic to receive waypoints
    rospy.Subscriber('/m4nav/flight_waypoints', Path, path_callback)

    # Subscribe to the state and pose topics
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)

    rate = rospy.Rate(10)  # Update rate of 10 Hz

    while not rospy.is_shutdown():
        if not current_state.connected:
            rospy.loginfo("Waiting for connection...")
        else:
            rospy.loginfo("Connected to MAVROS.")
            break

        rate.sleep()

    while not rospy.is_shutdown():
        if not waypoint_mission_sent:
            rospy.loginfo("Waiting for path message...")
        else:
            if current_state.mode != 'AUTO.MISSION':
                rospy.loginfo("Waiting for AUTO.MISSION mode...")
            elif not current_state.armed:
                rospy.loginfo("Waiting for vehicle to be armed...")
            elif current_pose.pose.position.z < 0.95 * target_altitude:
                rospy.loginfo("Waiting for vehicle to reach takeoff altitude...")
            else:
                # Check if the mission is completed
                # You can use your own logic here to determine mission completion
                mission_completed = False
                if mission_completed:
                    # Land the vehicle
                    if not land():
                        rospy.logerr("Failed to land the vehicle!")
                        return

                    # Disarm the vehicle
                    if not disarm():
                        rospy.logerr("Failed to disarm the vehicle!")
                        return

                    rospy.loginfo("Mission completed. Landing and disarming...")
                    break

        rate.sleep()

if __name__ == '__main__':
    main()
