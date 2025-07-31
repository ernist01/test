import time
from pymavlink import mavutil

# Initialize the MAVLink connection
# Replace '/dev/ttyACM0' with your Pixhawk serial port (it may vary)
# Baud rate is typically 57600 for Pixhawk USB connection
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for the heartbeat message to ensure we have communication
connection.wait_heartbeat()
print("Heartbeat from system (system ID: {})".format(connection.target_system))

# Define PWM values for the motors (adjust as needed)
# The PWM range for Pixhawk is typically from 1000 to 2000 microseconds
# These are standard values; you can adjust them as needed
motor_pwm_values = [1500, 1500, 1500, 1500]  # Set all motors to 1500 PWM (neutral throttle)

# Function to send PWM commands to motors
def send_pwm_commands():
    # Send each motor PWM signal to Pixhawk
    for i, pwm in enumerate(motor_pwm_values):
        # Motor number starts from 1 and goes up to 4 for Pixhawk
        connection.mav.rc_channels_override_send(
            connection.target_system, connection.target_component,
            pwm if i == 0 else 0,  # Channel 1 (motor 1)
            pwm if i == 1 else 0,  # Channel 2 (motor 2)
            pwm if i == 2 else 0,  # Channel 3 (motor 3)
            pwm if i == 3 else 0,  # Channel 4 (motor 4)
            0,  # Channel 5 (unused)
            0,  # Channel 6 (unused)
            0,  # Channel 7 (unused)
            0   # Channel 8 (unused)
        )
        time.sleep(0.01)  # 100Hz = 0.01s between commands

# Run the loop to send commands at 100Hz
try:
    while True:
        send_pwm_commands()
        time.sleep(0.01)  # Ensure we're sending at 100Hz
except KeyboardInterrupt:
    print("Program interrupted.")
