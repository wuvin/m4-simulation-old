#!/usr/bin/env python3

import rospy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path,Odometry
from grid_3d_nav.msg import PoseStampedwithflag,GlobalPath
from geometry_msgs.msg import PoseStamped,Quaternion
import numpy as np
from heapq import heappop, heappush
from tf.transformations import quaternion_from_euler
from math import sqrt, atan2 ,ceil,isnan

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
        #locomotion mode wheeled = 0, aerial = 1 
        self.locomotion_mode_wheeled = 0
        self.locomotion_mode_flying = 1
        self.robot_radius = 24
        self.clearance = 1 # 1 m clearance from the obstacles 
        

        self.distance_threshold = 0.2  # Adjust the threshold value as needed as we need to compute goal to current

        # flag to indicate whether a new goal has been received
        self.new_goal_received = False

        # create a publisher for the path
        self.path_pub = rospy.Publisher('/global/path', GlobalPath, queue_size=1)

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

        data1 = data[1]
        data0 = data[0]
        #print(data1)
        costmap_array = np.array(data1.data)
        elevation_array = np.array(data0.data)
        #np.save('flatted.npy',numpy_array)
        #costmap = numpy_array.reshape((int(self.height / self.resolution), int(self.width / self.resolution)))

        costmap = costmap_array.reshape((int(self.height / self.resolution), int(self.width / self.resolution)),order="F")

        #elevation 
        elevation = elevation_array.reshape((int(self.height / self.resolution), int(self.width / self.resolution)),order="F")

        #TO GET A NUMPY ARRAY FOR system
        np.save('elevation.npy',elevation)
        np.save('costmap.npy',costmap)
        
        self.costmap = costmap
        self.elevation = elevation

        #neighbor_cost = self.costmap[27,31]

        #print(neighbor_cost)

        # set the origin and resolution of the costmap
        self.origin = np.array([msg.info.pose.position.x, msg.info.pose.position.y])
        self.resolution = msg.info.resolution

        # create a Path message
        path_msg = GlobalPath()

        # set the header of the Path message
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = msg.info.header.frame_id

        # run A* algorithm
        #print(self.start)
        path = self.a_star(self.start, self.goal)

        #print(path)

        x_values = np.array([point[0][0] for point in path])
        y_values = np.array([point[0][1] for point in path])
        z_values = np.array([point[1] for point in path])
        flag_values = np.array([point[2] for point in path])

        #path[1,1]
        #new_path = np.array(path,dtype=np.float64)
        new_path = np.column_stack((x_values.astype(float), y_values.astype(float), z_values.astype(float), flag_values.astype(float)))
        new_path = np.round(new_path, decimals=3)

        #print(new_path)

        #print(new_path)
        path = self.take_off_landing_addition(new_path)
        #path = self.prune_path(new_path)
        print(path)
        np.save('3d_path.npy',path)

        #final_path = np.zeros((len(path), 3))

        #initial index
        #final_path[0] = np.append(path[0], 0.0)
        
        # for i in range(1, len(path)):
        #     current_point = path[i]
        #     print(current_point)
        #     previous_point = path[i-1]

        #     # Calculate the heading (orientation) using atan2
        #     delta_x = current_point[0] - previous_point[0]
        #     delta_y = current_point[1] - previous_point[1]
        #     heading = atan2(delta_y, delta_x)

        #     # Assign the heading to the current point

        #     final_path[i] = np.append(current_point, heading)

        #print(final_path)

        # create a PoseStamped message for each point on the path
        for point in path:
            pose_msg = PoseStampedwithflag()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = point[0]
            #print(point[0])
            pose_msg.pose.position.y = point[1]
            pose_msg.pose.position.z = point[2]
            pose_msg.flag = int(point[3])
            #print(self.heading_to_quaternion(point[2]))
            #quaternion = self.heading_to_quaternion(point[2])
            #pose_msg.pose.orientation= Quaternion(*quaternion)
            #print(pose_msg.pose.orientation)
            path_msg.poses.append(pose_msg)
        
        # publish the Path message
        #print(path_msg)
        self.path_pub.publish(path_msg)
        rospy.loginfo('path is published')
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
    
    def take_off_landing_addition(self,raw_path):

        # for straight takeoff and match the next flight point level 
        prev_flag = None
        inserted_rows = 0
        counter=0

        full_path= raw_path

        for i in range(len(raw_path)):
            current_flag = int(raw_path[i, 3])  # Access the flight or walk mode 4th row
            
            if prev_flag is not None and prev_flag != current_flag:
                prev_element = raw_path[i - 1]  # Access the previous whole element
                current_element = raw_path[i] # Access the current whole element
                counter += 1
        
        # Perform takeoff point creation
                if prev_flag == 0 and current_flag == 1:
                    extra_element = np.array([prev_element[0], prev_element[1], current_element[2], 1]) 
                    full_path = np.insert(full_path, (i+counter)-1, extra_element, axis=0) # add the current x,y 
                    #print(counter, (i+counter))

                # Perform landing point creation
                elif prev_flag == 1 and current_flag == 0:
                    extra_element = np.array([current_element[0], current_element[1], prev_element[2], 0])
                    
                    full_path = np.insert(full_path, (i+counter)-1, extra_element, axis=0)  # Create a 4x1 element with the new landing hovering
                    #print(counter, (i+counter))
            
            prev_flag = current_flag

        #print(full_path)

        return full_path
    


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
    
    def get_max_elevation(self, x, y):

        center_index = (x,y)

        # Size of the square region (odd value for simplicity)
        region_size = self.robot_radius

        # Extract the square region using array indexing around the current point\
        start_x = max(x - region_size // 2, 0)
        end_x = min(x + region_size // 2 + 1, self.elevation.shape[0])
        start_y = max(y - region_size // 2, 0)
        end_y = min(y + region_size // 2 + 1, self.elevation.shape[1])
        region_of_interest = self.elevation[start_x:end_x, start_y:end_y]
        max_elev = np.nanmax(region_of_interest)

        return max_elev    



    def heuristic(self, a, b):
        if a is None or b is None:
            return float('inf') 
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    def change_in_elevation(self, a, b):
        if a is None or b is None:
            return float('inf') 
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def a_star(self,start, goal):
        open_list = [(0, tuple(start))] # start of priority list 
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
                    current ,z_value, flag= came_from[tuple(current)]
                    path.append((current,z_value,flag))
                path.reverse()
                #assuming goal is always ground 
                path[-1] = (path[-1], 0, 0)
                return path

            for neighbor in self.get_neighbors(current):
                #print(self.world_to_grid(neighbor))
                #print(neighbor)
                x, y = self.world_to_grid(neighbor)
                x0, y0 = self.world_to_grid(current)

                #print(x,y)
                neighbor_cost = self.costmap[x,y]
                elevation_neighbor = self.elevation[x,y]
                elevation_current = self.elevation[x0,y0]

                #adding elevation as 100 if the output isnan
                if  isnan(elevation_current):
                    elevation_current = 100
                if  isnan(elevation_neighbor):
                    elevation_neighbor = 100
                    

                #elevation in meters need to convert to grid parameters
                change_in_elevation = round((abs(elevation_current - elevation_neighbor))/self.resolution)
                #print(change_in_elevation)
                #if isnan(change_in_elevation) :
                    #print(change_in_elevation)
                    #change_in_elevation =0
                    #change_in_elevation=ceil(change_in_elevation)
                #print(change_in_elevation)

                if neighbor_cost == 60:
                        flag = self.locomotion_mode_flying

                        z_value = self.get_max_elevation(x,y) +self.clearance
                        #z_value = elevation_neighbor + self.clearance 
                        tentative_g_score = g_score[tuple(current)] + self.heuristic(current, neighbor) + neighbor_cost*change_in_elevation + 1 + neighbor_cost 
                else :
                        flag = self.locomotion_mode_wheeled
                        z_value = elevation_neighbor
                        tentative_g_score = g_score[tuple(current)] + self.heuristic(current, neighbor) + neighbor_cost
                
                #print(neighbor)
                #print(neighbor_cost)

                if tuple(neighbor) not in g_score or tentative_g_score < g_score[tuple(neighbor)]:
                    came_from[tuple(neighbor)] = current , z_value, flag
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
        #print(x)W
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
                    wor= self.grid_to_world(neighbor_x, neighbor_y)
                    #print(wor)
                    neighbors.append(self.grid_to_world(neighbor_x, neighbor_y))
        return neighbors
    
if __name__ == '__main__':
    planner = global_planner()
    rospy.spin()
