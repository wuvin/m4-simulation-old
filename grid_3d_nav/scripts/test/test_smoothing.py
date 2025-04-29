import numpy as np

def is_collision_free(point, elevation_map, collision_threshold):
    # Check if the given point is within the bounds and collision-free based on the elevation map
    x, y = point
    if x < 0 or x >= elevation_map.shape[0] or y < 0 or y >= elevation_map.shape[1]:
        return False
    elevation = elevation_map[x, y]
    return elevation <= collision_threshold

def smooth_path(initial_path, elevation_map, collision_threshold, resolution):
    smoothed_path = [initial_path[0]]  # Start with the initial point

    for i in range(len(initial_path) - 1):
        current_point = initial_path[i]
        next_point = initial_path[i + 1]

        if is_collision_free(next_point, elevation_map, collision_threshold):
            # If the next point is collision-free, add it to the smoothed path
            smoothed_path.append(next_point)

    # Apply smoothing algorithm (e.g., B-spline or polynomial interpolation) to the smoothed path

    return smoothed_path

# Example usage
initial_path = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
elevation_map = np.array([[1.2, 2.1, 0.9],
                          [1.1, 0.8, 0.7],
                          [1.4, 1.3, 1.2],
                          [2.5, 2.3, 2.6],
                          [3.1, 2.9, 3.3]])
collision_threshold = 2.0
resolution = 1

smoothed_path = smooth_path(initial_path, elevation_map, collision_threshold, resolution)

print("Initial Path:", initial_path)
print("Smoothed Path:", smoothed_path)
