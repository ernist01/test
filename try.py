import time
from pymavlink import mavutil

# Connect to Pixhawk over serial (adjust the port based on your setup)
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)  # Use correct serial port

# Wait for the heartbeat to establish communication
connection.wait_heartbeat()
print(f"Heartbeat received from system {connection.target_system}, component {connection.target_component}")

# Set PWM parameters
motor_pwm_channel = 1  # Channel number for the motor (1-8 typically)
motor_pwm_value = 1500  # Set the PWM value (1000-2000) based on motor requirements

# The goal is to send a PWM signal at 100Hz (10ms interval)
while True:
    # Send PWM signal to the motor
    connection.mav.command_long_send(
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command to set servo (PWM output)
        0,  # Confirmation
        motor_pwm_channel,  # Which PWM channel to control
        motor_pwm_value,  # PWM value to set (0-2000)
        0, 0, 0, 0, 0  # Additional parameters (unused here)
    )
    
    # Sleep for 10ms to maintain 100Hz frequency
    time.sleep(0.01)  # 100Hz means sending the command every 10ms
