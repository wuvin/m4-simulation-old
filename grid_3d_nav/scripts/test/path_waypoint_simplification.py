import numpy as np

def distance(point1, point2):
    # Calculate the Euclidean distance between two 3D points
    return np.linalg.norm(np.array(point1) - np.array(point2))

def perpendicular_distance(point, line_start, line_end):
    # Calculate the perpendicular distance from a point to a line formed by two 3D points
    line_vector = np.array(line_end) - np.array(line_start)
    point_vector = np.array(point) - np.array(line_start)
    projection = np.dot(point_vector, line_vector) / np.dot(line_vector, line_vector)
    closest_point = np.array(line_start) + projection * line_vector
    return distance(point, closest_point)

def simplify_path(points, tolerance, elevation_map):
    if len(points) < 3:
        return points

    # Find the point with the maximum perpendicular distance
    max_distance = 0
    max_index = 0
    line_start = points[0]
    line_end = points[-1]

    for i in range(1, len(points) - 1):
        dist = perpendicular_distance(points[i], line_start, line_end)
        if dist > max_distance:
            max_distance = dist
            max_index = i

    simplified_points = []

#collision checking with the elevation map 
def is_collision_free(path, elevation_map, clearance):
    for i in range(len(path)-1):
        start_x, start_y, start_z = path[i]
        end_x, end_y, end_z = path[i+1]
        
        num_samples = int(max(abs(end_x - start_x), abs(end_y - start_y))) + 1
        x_step = (end_x - start_x) / num_samples
        y_step = (end_y - start_y) / num_samples
        
        for j in range(num_samples):
            x = start_x + j * x_step
            y = start_y + j * y_step
            z = start_z + (j * x_step + j * y_step) * (end_z - start_z) / (end_x - start_x + end_y - start_y)
            
            if z > elevation_map[x, y] + clearance:
                return False
    
    return True

    # If the maximum distance is greater than the tolerance, recursively simplify the path
    if max_distance > tolerance:
        # Simplify the path before and after the point with maximum distance
        simplified_points += simplify_path(points[:max_index + 1], tolerance, elevation_map)
        simplified_points += simplify_path(points[max_index:], tolerance, elevation_map)
    else:
        # Add the start and end points to the simplified path
        simplified_points.append(points[0])
        simplified_points.append(points[-1])

    return simplified_points

def is_collision_free(point, elevation_map, clearance):
    # Check if a point is collision-free based on the elevation map and clearance
    x, y, z = point
    elevation = elevation_map[x, y]
    if elevation + clearance <= z:
        return True
    else:
        return False

def smooth_path(path, elevation_map, clearance, tolerance):
    smoothed_path = [path[0]]  # Start with the initial point

    for i in range(1, len(path) - 1):
        current_point = path[i]
        next_point = path[i + 1]

        # Check for collision with the next point
        if is_collision_free(next_point, elevation_map, clearance):
            smoothed_path.append(next_point)

    smoothed_path.append(path[-1])  # Add the goal point

    return smoothed_path

# Example usage:
path = [(0, 0, 0), (1, 0, 1), (2, 0, 2), (3, 0, 3), (4, 0, 2), (5, 0, 1), (6, 0, 0)]
elevation_map = np.array([[0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2]])
clearance = 0.5
tolerance = 0.1

simplified_path = simplify_path(path, tolerance, elevation_map)
smoothed_path = smooth_path(simplified_path, elevation_map, clearance, tolerance)

print("Simplified Path:", simplified_path)
print("Smoothed Path:", smoothed_path)
