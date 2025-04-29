#!/usr/bin/env python3

import rospy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Quaternion
import numpy as np
from heapq import heappop, heappush
from tf.transformations import quaternion_from_euler
from math import sqrt, atan2

class global_planner:
    def __init__(self):
        # initialize ROS node
        rospy.init_node('a_star_planner')

        # subscribe to the costmap topic
        rospy.Subscriber('/elevation_mapping/costmap', GridMap, self.costmap_callback)
        #m4
        rospy.Subscriber('/my_odom', Odometry, self.odom_callback)
        #turtlebot
        #rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)

        # set initial values for start and goal
        
        self.start = None
        self.goal = None

        self.distance_threshold = 0.2  # Adjust the threshold value as needed as we need to compute goal to current

        # flag to indicate whether a new goal has been received
        self.new_goal_received = False

        # create a publisher for the path
        self.path_pub = rospy.Publisher('/global/path', Path, queue_size=1)

        # wait for costmap to be received
        while not hasattr(self, 'costmap'):
            rospy.sleep(0.1)

    def costmap_callback(self, msg):

        # check if a new goal has been received
        if not self.new_goal_received:
            return
        
            # Access the metadata
        self.width = msg.info.length_x
        self.height = msg.info.length_y
        self.resolution = msg.info.resolution
        layers = msg.layers

        print("GridMap Information:")
        print("Width: ", self.width)
        print("Height: ", self.height)
        print("Resolution: ", self.resolution)
        print("Layers: ", layers)

        # Access the data
        data = msg.data

        #data123=GridMap(msg)
        #print(data123)

        data1 = data[1]
        #print(data1)
        numpy_array = np.array(data1.data)
        #np.save('flatted.npy',numpy_array)
        #costmap = numpy_array.reshape((int(self.height / self.resolution), int(self.width / self.resolution)))

        costmap = numpy_array.reshape((int(self.height / self.resolution), int(self.width / self.resolution)),order="F")

        np.save('1array.npy',costmap)
        
        self.costmap = costmap

        #neighbor_cost = self.costmap[27,31]

        #print(neighbor_cost)

        # set the origin and resolution of the costmap
        self.origin = np.array([msg.info.pose.position.x, msg.info.pose.position.y])
        self.resolution = msg.info.resolution

        # create a Path message
        path_msg = Path()

        # set the header of the Path message
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = msg.info.header.frame_id

        # run A* algorithm
        #print(self.start)
        path = self.a_star(self.start, self.goal)

        #path[1,1]
        new_path = np.array(path)

        #print(new_path)

        path = self.prune_path(new_path)
        #print(path)


        final_path = np.zeros((len(path), 3))

        #initial index
        final_path[0] = np.append(path[0], 0.0)
        
        for i in range(1, len(path)):
            current_point = path[i]
            print(current_point)
            previous_point = path[i-1]

            # Calculate the heading (orientation) using atan2
            delta_x = current_point[0] - previous_point[0]
            delta_y = current_point[1] - previous_point[1]
            heading = atan2(delta_y, delta_x)

            # Assign the heading to the current point

            final_path[i] = np.append(current_point, heading)

        #print(final_path)

        # create a PoseStamped message for each point on the path
        for point in final_path:
            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = point[0]
            #print(point[0])
            pose_msg.pose.position.y = point[1]
            pose_msg.pose.position.z = 0.0
            #print(self.heading_to_quaternion(point[2]))
            quaternion = self.heading_to_quaternion(point[2])
            pose_msg.pose.orientation= Quaternion(*quaternion)
            #print(pose_msg.pose.orientation)
            path_msg.poses.append(pose_msg)
        
        # publish the Path message
        #print(path_msg)
        self.path_pub.publish(path_msg)
        self.new_goal_received = False
    
    def odom_callback(self, odom_msg):
        # Extract the position from the odometry message
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        self.start = np.array([x, y])
        #print(self.start)
        
    def goal_callback(self, msg):
        # get the goal coordinates from the move_base/point message
        x = msg.pose.position.x
        y = msg.pose.position.y
        #print(x,y)
        self.goal = np.array([x, y])
        #print(x,y)
        self.new_goal_received = True
    
    def heading_to_quaternion(self,heading):
        quaternion = quaternion_from_euler(0, 0, heading)
        return quaternion

    def prune_path(self,path, epsilon=1e-5):
        """
        Returns pruned path.
        """
        def point(p):
            return np.array([p[0], p[1], 1.]).reshape(1, -1)

        def collinearity_check(p1, p2, p3):
            m = np.concatenate((p1, p2, p3), 0)
            det = np.linalg.det(m)
            return abs(det) < epsilon

        pruned_path = [p for p in path]
        i = 0
        while i < len(pruned_path) - 2:
            p1 = point(pruned_path[i])
            print(p1)
            p2 = point(pruned_path[i + 1])
            p3 = point(pruned_path[i + 2])

            if collinearity_check(p1, p2, p3):
                pruned_path = np.delete(pruned_path, i + 1, axis=0)
                #pruned_path.remove(pruned_path[i + 1]).all
            else:
                i += 1

        return pruned_path
    


    def world_to_grid(self, pos):
        # Define the homogeneous transformation matrix T12
        a= (self.height/2)
        b= (self.width/2)

        self.map_x = a + self.origin[0]
        self.map_y = b + self.origin[1]
        #World to grid 
        T12 = np.array([[-1, 0, a],
                        [0, -1, b],
                        [0, 0,  1]])
        #world to grid dividing by resolution
        self.wtg_homo_T = T12

        #print(pos)
        

        values =np.array([pos[0], pos[1], 1])

        # Perform the transformation
        grid_xy = (np.dot(self.wtg_homo_T, values))

        #converting to grid cell value
        x = (grid_xy[0]/self.resolution).astype(int)
        y = (grid_xy[1]/self.resolution).astype(int)
        #print(x,y)
        #return ((pos - self.origin) / self.resolution).astype(int)
        return x,y

    def grid_to_world(self, x, y):

        gtw_homo_T= np.linalg.inv(self.wtg_homo_T)
        girdxy = np.array([x*self.resolution,y*self.resolution,1])
        world_xy = np.dot(gtw_homo_T,girdxy)
        worldxy=world_xy[:2]
        #print(world)
        #return self.origin + np.array([x, y]) * self.resolution
        return worldxy



    def heuristic(self, a, b):
        if a is None or b is None:
            return float('inf') 
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    def a_star(self,start, goal):
        open_list = [(0, start)]
        #print(open_list)
        came_from = {}
        g_score = {tuple(start): 0}
        f_score = {tuple(start): self.heuristic(start, goal)}
        #print(f_score)

        while open_list:
            current = heappop(open_list)[1]

            if self.heuristic(current, goal) <= self.distance_threshold:
                # Goal reached or within the vicinity
                path = [current]
                while tuple(current) in came_from:
                    current = came_from[tuple(current)]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                #print(self.world_to_grid(neighbor))
                #print(neighbor)
                x, y = self.world_to_grid(neighbor)

                #print(x,y)
                neighbor_cost = self.costmap[x,y]
                tentative_g_score = g_score[tuple(current)] + self.heuristic(current, neighbor) + neighbor_cost

                #print(neighbor)
                #print(neighbor_cost)

                if tuple(neighbor) not in g_score or tentative_g_score < g_score[tuple(neighbor)]:
                    came_from[tuple(neighbor)] = current
                    g_score[tuple(neighbor)] = tentative_g_score
                    f_score[tuple(neighbor)] = tentative_g_score + self.heuristic(neighbor, goal)

                    #print(f_score[tuple(neighbor)])

                    #print(open_list)
                    #print(f_score[tuple(neighbor)])
                    #print(tuple(neighbor))

                    heappush(open_list, (f_score[tuple(neighbor)], tuple(neighbor)))

        return []

    def get_neighbors(self, current):
        x, y = self.world_to_grid(current)

        #print(x,y)
        #print(x)
        #print(y)
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor_x = x + dx
                neighbor_y = y + dy
                if (0 <= neighbor_x < self.costmap.shape[0] and
                        0 <= neighbor_y < self.costmap.shape[1] ):
                    #print(neighbor_x,neighbor_y)
                    cell_cost = self.costmap[neighbor_x,neighbor_y]
                    #print(cell_cost)
                    if cell_cost == 60:
                        continue  # Skip blocked cells
                    #print(cell_cost)
                    wor= self.grid_to_world(neighbor_x, neighbor_y)
                    #print(wor)
                    neighbors.append(self.grid_to_world(neighbor_x, neighbor_y))
        return neighbors
    
if __name__ == '__main__':
    planner = global_planner()
    rospy.spin()
