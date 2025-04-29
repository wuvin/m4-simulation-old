#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import  Pose,PoseStamped, TwistStamped
from std_msgs.msg import Header

class QuadrotorController:

    def __init__(self):

        rospy.init_node('quadrotor_controller')

        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped() 

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_callback)

        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_mode(custom_mode='GUIDED')

        #changing the frame to Body NED frame   
        rospy.wait_for_service('/mavros/setpoint_position/mav_frame')
        self.set_mode = rospy.ServiceProxy('/mavros/setpoint_position/mav_frame', SetMavFrame)
        self.set_mode(mav_frame = '8')


        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm(True)

    def pose_callback(self, msg):
        self.current_pose = msg

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def takeoff(self):
        rate = rospy.Rate(10)
        rospy.wait_for_service('/mavros/cmd/takeoff')
        takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        takeoff_client(altitude= '1')
        takeoff_height = 10
        setpoint_msg = PoseStamped()
        setpoint_msg.header.frame_id = ''
        setpoint_msg.pose.position = self.current_pose.pose.position
        setpoint_msg.pose.orientation = self.current_pose.pose.orientation

        while abs(self.current_pose.pose.position.z) < takeoff_height:
            setpoint_msg.pose.position.z += 0.1
            self.setpoint_pub.publish(setpoint_msg)
            rate.sleep()

    def hover(self, duration):
        rate = rospy.Rate(10)
        setpoint_msg = PoseStamped()
        setpoint_msg.header.frame_id = ''
        setpoint_msg.pose.position = self.current_pose.pose.position
        setpoint_msg.pose.orientation = self.current_pose.pose.orientation

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.setpoint_pub.publish(setpoint_msg)
            rate.sleep()

    def newpoint(self,x,y,z):
        rate = rospy.Rate(10)
        setpoint_msg = PoseStamped()
        setpoint_msg.header.frame_id = ''
        setpoint_msg.header.stamp= rospy.Time.now()
        new_position = setpoint_msg.pose.position
        new_position.x = x
        new_position.y = y
        new_position.z = z
        setpoint_msg.pose.orientation = self.current_pose.pose.orientation
        self.setpoint_pub.publish(setpoint_msg)


    def land(self):
        rate = rospy.Rate(10)
        land_height = 0.0
        setpoint_msg = PoseStamped(
            header=Header(frame_id=''),
            pose=Pose(
                position=self.current_pose.pose.position,
                orientation=self.current_pose.pose.orientation
            )
        )

        while self.current_pose.pose.position.z > land_height:
            setpoint_msg.pose.position.z -= 0.1
            self.setpoint_pub.publish(setpoint_msg)
            rate.sleep()

        self.arm(False)

if __name__ == '__main__':
    quadrotor_controller = QuadrotorController()
    rospy.sleep(1)
    #TAKEOFF
    quadrotor_controller.takeoff()
    #Hover
    quadrotor_controller.hover(10)
    #SET POINT FOR DRONE TO GO TO FROM ITS CURRENT LOCATION
    quadrotor_controller.newpoint(4,4,4)
    #LAND
    quadrotor_controller.land()
