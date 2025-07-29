from pymavlink import mavutil
import time

# Define the serial connection to Pixhawk
# Adjust the connection string for your setup, e.g., '/dev/ttyUSB0' for USB, or '/dev/ttyAMA0' for serial.
# Baudrate is typically 57600 for telemetry or USB.
connection_string = '/dev/ttyUSB0'  # Replace with your Pixhawk's port
baud_rate = 57600  # Typical baud rate for Pixhawk

# Establish connection to Pixhawk
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()  # Wait for the first heartbeat (Pixhawk connection confirmation)
print("Connected to Pixhawk")

# Set mode to GUIDED (autonomous control)
master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)

# Arm the vehicle (enable ESC and motor control)
master.arducopter_arm()
print("Arming the vehicle...")

# Give the vehicle some time to arm
time.sleep(2)

# Send throttle (PWM signal) to control the ESC
# The value is from 1000 to 2000 microseconds, where:
# - 1000 is idle (motor off or minimum speed)
# - 1500 is neutral (half throttle)
# - 2000 is maximum throttle (full speed)

# Setting throttle to minimum (1000 microseconds)
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0
)
print("Throttle set to minimum")

# Wait for 3 seconds at minimum throttle
time.sleep(3)

# Set throttle to maximum (2000 microseconds)
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 2000, 0, 0, 0
)
print("Throttle set to maximum")

# Wait for 3 seconds at maximum throttle
time.sleep(3)

# Set throttle back to minimum (1000 microseconds)
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111001000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0
)
print("Throttle set to minimum again")

# Disarm the vehicle (disable ESC)
master.arducopter_disarm()
print("Disarming the vehicle...")

# Close connection
master.close()
