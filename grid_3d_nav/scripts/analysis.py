import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from grid_3d_nav.msg import PoseStampedwithflag,GlobalPath

class PathMetricsCalculator:
    def __init__(self):
        self.total_distance = 0.0
        self.total_power = 0.0
        self.last_pose = None
        self.last_z = None
        self.velocity = 0.0  # Initial velocity
        self.cost_flight_to_walk = 10.0  # Cost to change from flight to walk (adjust as needed)
        rospy.Subscriber('/global/path', GlobalPath, self.path_callback)

    def path_callback(self, path_msg):
        for pose_stamped in path_msg.poses:
            if self.last_pose is not None:
                distance = self.calculate_distance(self.last_pose.pose, pose_stamped.pose)
                self.total_distance += distance
                
                # Calculate time to travel between poses based on velocity
                time_to_travel = distance / self.velocity
                
                # Calculate power consumption based on velocity and z-value
                power_consumption = self.velocity
                if pose_stamped.pose.position.z == 0:  # z = 0
                    power_consumption *= 1.0  # Modify power consumption factor
                elif 1 <= pose_stamped.pose.position.z <= 2:  # z = 1 to 2
                    power_consumption *= 1.5  # Modify power consumption factor
                else:  # z = 3 and above
                    power_consumption *= 2.0  # Modify power consumption factor
                
                # Account for cost of changing from flight to walk
                if pose_stamped.pose.position.z == 0 and self.last_z != 0:
                    power_consumption += self.cost_flight_to_walk
                
                # Accumulate total power consumption
                self.total_power += power_consumption * time_to_travel
                    
            self.last_pose = pose_stamped
            self.last_z = pose_stamped.pose.position.z

    def calculate_distance(self, p1, p2):
        dx = p2.position.x - p1.position.x
        dy = p2.position.y - p1.position.y
        dz = p2.position.z - p1.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance

if __name__ == '__main__':
    rospy.init_node('path_metrics_calculator')
    metrics_calculator = PathMetricsCalculator()
    
    # Set the velocities for different segments of the path
    metrics_calculator.velocity = 1.0  # Set an appropriate velocity value
    rospy.sleep(5)  # Simulate movement at this velocity
    
    # Print the results
    print("Total 3D Path Distance:", metrics_calculator.total_distance, "meters")
    print("Total Power Consumption:", metrics_calculator.total_power, "Watt-seconds")
    
    rospy.spin()
