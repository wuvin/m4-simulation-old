import rospy
from gazebo_msgs.srv import JointRequest, JointRequestRequest

class JointTypeSwitcher:
    def __init__(self):
        self.joint_name = 'your_joint_name'
        self.is_fixed = False

        # Create ROS service proxies
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self.get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', JointRequest)
        rospy.wait_for_service('/gazebo/set_joint_properties')
        self.set_joint_properties = rospy.ServiceProxy('/gazebo/set_joint_properties', JointRequest)

    def switch_joint_type(self, is_fixed):
        # Get current joint properties
        try:
            response = self.get_joint_properties(self.joint_name)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get joint properties: %s", e)
            return False

        # Set new joint properties
        request = JointRequestRequest()
        request.joint_name = self.joint_name
        request.ode_joint_config = response.ode_joint_config
        request.ode_joint_config.type = 'fixed' if is_fixed else 'revolute'
        try:
            self.set_joint_properties(request)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to set joint properties: %s", e)
            return False

        self.is_fixed = is_fixed
        return True

if __name__ == '__main__':
    rospy.init_node('joint_type_switcher')
    switcher = JointTypeSwitcher()

    # Example usage: switch joint type every 5 seconds
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        is_fixed = not switcher.is_fixed
        switcher.switch_joint_type(is_fixed)
        rospy.loginfo("Joint type switched to %s", 'fixed' if is_fixed else 'revolute')
        rate.sleep()