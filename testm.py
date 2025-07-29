from pymavlink import mavutil
import time

# 1. Connect to Pixhawk via USB (adjust the connection string if needed)
# /dev/ttyACM0 is the common connection string for USB connections to Pixhawk.
connection_string = '/dev/ttyACM0'  # Adjust according to your setup
baud_rate = 57600  # Standard baud rate for Pixhawk
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# 2. Wait for the heartbeat to verify connection
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system ID {master.target_system}, component ID {master.target_component})")

# 3. Arm the vehicle (basic arming command)
print("Arming vehicle...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)

# Wait to make sure the vehicle is armed
time.sleep(5)

# 4. Send a simple velocity command to move the vehicle
# (For example, a small forward velocity of 1 m/s in the x-axis)
print("Sending velocity command...")
master.mav.set_position_target_local_ned_send(
    0,  # Time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Flags to enable position and velocity
    0, 0, 0,  # Velocity in x, y, z axes (in meters per second)
    1, 0, 0,  # Forward velocity (1 m/s in the x-direction)
    0, 0, 0   # No change in yaw or other parameters
)

# Wait a little to ensure the command is sent
time.sleep(2)

# 5. Disarm the vehicle after sending the command
print("Disarming vehicle...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
)

# 6. Close the connection
master.close()
print("Communication finished.")
