import numpy as np
import time
from filterpy.kalman import ExtendedKalmanFilter
from scipy.spatial import distance
import heapq
import mavutil
import pid

# Define your MAVLink connection to Pixhawk
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Set up PID controllers for depth and orientation
depth_pid = pid.PID(Kp=1.0, Ki=0.1, Kd=0.05)
attitude_pid = pid.PID(Kp=1.0, Ki=0.1, Kd=0.05)

# Set the target depth and orientation (desired state)
target_depth = 5.0  # meters
target_roll = 0.0   # neutral roll
target_pitch = 0.0  # neutral pitch

# Define the ROV's position (x, y, z)
rov_position = np.array([0.0, 0.0, 5.0])  # [x, y, z]
rov_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]

# Map size (for A* pathfinding)
map_size = (100, 100)

# Initialize EKF for sensor fusion (state: [x, y, vx, vy])
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=4)
ekf.P *= 10  # Initial uncertainty
ekf.R *= 0.1  # Sensor measurement noise

# Initialize A* grid
obstacle_map = np.zeros((map_size[0], map_size[1]))  # 0 for free space, 1 for obstacles

# Function to get sensor data (dummy values for this example)
def get_sensor_data():
    # Replace with actual sensor data fetching functions
    depth = 5.0  # Dummy depth value
    pitch, roll, yaw = 0.0, 0.0, 0.0  # Dummy IMU data
    flow = np.array([0.1, 0.1])  # Dummy optical flow data (velocity)
    return depth, pitch, roll, yaw, flow

# A* Algorithm for Pathfinding
def a_star(start, goal, obstacles):
    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def neighbors(node):
        x, y = node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            if 0 <= x + dx < map_size[0] and 0 <= y + dy < map_size[1]:
                if obstacles[x + dx][y + dy] == 0:  # Check for free space
                    yield (x + dx, y + dy)

    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    open_set = []
    heapq.heappush(open_set, (f_score[start], start))
    
    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in neighbors(current):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # Path not found

# Artificial Potential Fields for Obstacle Avoidance
def artificial_potential_fields(position, goal, obstacles):
    repulsive_field = np.zeros_like(position)
    attractive_field = goal - position

    # Repulsive force from obstacles (simple model)
    for obs in obstacles:
        dist = np.linalg.norm(position - obs)
        if dist < 5.0:
            repulsive_field += (position - obs) / dist**2
    
    total_field = attractive_field - repulsive_field
    return total_field

# EKF update function
def ekf_update(rov_position, rov_velocity, sensor_data):
    z = np.array([sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3]])  # Depth, IMU data
    ekf.predict()
    ekf.update(z)
    rov_position[0:2] += rov_velocity  # Update position based on velocity
    return rov_position

# Control loop
def control_rover(depth, pitch, roll):
    # Control depth
    depth_error = target_depth - depth
    depth_adjustment = depth_pid.update(depth_error)

    # Control orientation (roll and pitch)
    roll_error = target_roll - roll
    pitch_error = target_pitch - pitch
    roll_adjustment = attitude_pid.update(roll_error)
    pitch_adjustment = attitude_pid.update(pitch_error)

    # Send commands to Pixhawk via MAVLink (simple example)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500 + depth_adjustment,  # Adjust PWM values for thrusters
        1500 + roll_adjustment,
        1500 + pitch_adjustment
    )

def autonomous_navigation():
    start = (0, 0)  # Starting point (example)
    goal = (50, 50)  # Goal point (example)

    # Create a dummy obstacle map
    obstacles = [(30, 30), (40, 40), (25, 35)]  # Example obstacles

    # Start Autonomous Navigation
    while True:
        depth, pitch, roll, yaw, flow = get_sensor_data()

        # Perform sensor fusion using EKF
        rov_position = ekf_update(rov_position, flow, [depth, pitch, roll, yaw])

        # Use A* algorithm to find path
        path = a_star(start, goal, obstacle_map)
        if path is None:
            print("No path found!")
            break

        # Apply APF to avoid obstacles
        control_vector = artificial_potential_fields(rov_position, goal, obstacles)

        # Apply control logic
        control_rover(depth, pitch, roll)

        time.sleep(0.1)  # Loop frequency

if __name__ == '__main__':
    autonomous_navigation()
