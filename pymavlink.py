from dronekit import connect
import time

# Connect to Pixhawk via USB (replace with the correct port name found in step 3)
connection_string = '/dev/ttyACM0'  # Adjust this based on the output of ls /dev/serial/by-id/
baud_rate = 57600  # Pixhawk default baud rate

# Connect to the vehicle
print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(connection_string, wait_ready=True, baud=baud_rate)

# Print vehicle attributes
print(f"Vehicle: {vehicle}")
print(f"Vehicle System Status: {vehicle.system_status.state}")
print(f"Vehicle Location: {vehicle.location.global_frame}")

# Arm and takeoff example
vehicle.mode = "GUIDED"
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)
    
vehicle.simple_takeoff(10)  # Take off to 10 meters

# Wait for the vehicle to reach altitude
while vehicle.location.global_relative_frame.alt < 9.5:
    print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
    time.sleep(1)

print("Takeoff complete!")

# Close vehicle connection
vehicle.close()
