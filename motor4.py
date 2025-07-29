from pymavlink import mavutil
import time

# 1. Connect to Pixhawk via USB (adjust the connection string if needed)
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

# 4. Set motor to medium speed (1500 microseconds)
# The PWM signal at 1500 microseconds corresponds to about 50% throttle, which is medium speed
print("Setting motor to medium speed (1500 microseconds)...")
master.mav.set_position_target_local_ned_send(
    0,  # Time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111001000,  # Flags for velocity and position control
    0, 0, 0,  # No change in position
    0, 0, 0,  # No velocity command, only throttle
    1500,  # Medium throttle (PWM 1500 microseconds)
    0, 0   # No yaw or other changes
)

# Wait a little to ensure the medium speed is applied
time.sleep(5)

# 5. Disarm the vehicle after sending the command
print("Disarming vehicle...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
)

# 6. Close the connection
master.close()
print("Communication finished.")
