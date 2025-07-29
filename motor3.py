from pymavlink import mavutil
import time

# Define connection to Pixhawk via USB (adjust the connection string for your setup)
connection_string = '/dev/ttyACM0'  # For Linux, might be /dev/ttyUSB0 or other
baud_rate = 57600  # Typical baud rate for Pixhawk

# Establish connection to Pixhawk
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()  # Wait for heartbeat to confirm connection
print("Connected to Pixhawk")

# Arm the vehicle (this enables ESC and motor control)
master.arducopter_arm()
print("Arming vehicle...")

# Wait for 2 seconds to allow arming to complete
time.sleep(2)

# Set throttle to full (2000 microseconds = max speed)
print("Spinning motors at full speed (2000 microseconds)...")
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 2000, 0, 0, 0
)

# Wait for 5 seconds at full throttle (full speed)
time.sleep(5)

# Stop the motor (set throttle back to idle 1000 microseconds)
print("Stopping motor (1000 microseconds)...")
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0
)

# Disarm the vehicle (disables ESC and motor control)
time.sleep(2)
master.arducopter_disarm()
print("Disarming vehicle...")

# Close connection
master.close()
