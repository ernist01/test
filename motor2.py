from pymavlink import mavutil
import time

# Define the serial connection to Pixhawk via USB
# Change '/dev/ttyACM0' if your device is recognized differently (e.g., '/dev/ttyUSB0')
connection_string = '/dev/ttyACM0'  # Adjust this for your Pixhawk's USB port
baud_rate = 57600  # Typical baud rate for Pixhawk over USB

# Establish connection to Pixhawk
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()  # Wait for the first heartbeat (Pixhawk connection confirmation)
print("Connected to Pixhawk")

# Set mode to GUIDED (autonomous control mode)
master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)

# Arm the vehicle (this enables ESC and motor control)
master.arducopter_arm()
print("Arming the vehicle...")

# Wait for 2 seconds to allow arming to complete
time.sleep(2)

# Spin the motor at full throttle (2000 microseconds)
# This sends the maximum throttle command (full speed) to the ESC
print("Spinning motors at full speed...")
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 2000, 0, 0, 0
)

# Wait for 5 seconds at full throttle (full speed)
time.sleep(5)

# Set throttle back to minimum (1000 microseconds)
print("Stopping the motors (min speed)...")
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0
)

# Wait for 2 seconds
time.sleep(2)

# Disarm the vehicle (di
