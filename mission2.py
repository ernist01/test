import time
import math
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

from dronekit import connect, VehicleMode, mavutil

# === CONFIGURATION ===
PORT = '/dev/ttyAMA0'       # Pixhawk serial port on Pi (adjust if needed)
BAUD = 115200

TARGET_DEPTH = 1.5          # meters
DEPTH_TOLERANCE = 0.1       # meters
DESCEND_SPEED = 0.3         # m/s downward
FORWARD_SPEED = 0.5         # m/s forward
FORWARD_TIME = 10           # seconds

TARGET_YAW = 90             # degrees (east)
YAW_TOLERANCE = 5           # degrees
YAW_RATE = 0.3              # rad/s

# === INIT I2C + BNO08x ===
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

# === Connect to Pixhawk ===
print("Connecting to Pixhawk...")
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)


# === MAVLINK velocity command ===
def send_velocity(vx, vy, vz, yaw_rate=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Enable velocity + yaw rate
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def normalize_angle(angle):
    return angle % 360

def yaw_error(current, target):
    error = normalize_angle(target - current)
    if error > 180:
        error -= 360
    return error

# === ARM & SETUP ===
def arm_vehicle():
    print("Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)
    print("Arming...")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed and ready.")

# === Depth Control ===
def get_current_depth():
    if vehicle.rangefinder:
        return vehicle.rangefinder.distance
    return None

def dive_to_target_depth(target):
    print(f"Diving to {target:.2f}m...")
    while True:
        depth = get_current_depth()
        if depth is None:
            print("No depth sensor data.")
            break
        error = target - depth
        print(f"Depth: {depth:.2f} m, Error: {error:.2f}")
        if abs(error) <= DEPTH_TOLERANCE:
            send_velocity(0, 0, 0)
            break
        vz = DESCEND_SPEED if error > 0 else -DESCEND_SPEED
        send_velocity(0, 0, vz)
        time.sleep(0.5)
    send_velocity(0, 0, 0)
    print("Depth hold complete.")

# === Yaw Control ===
def get_current_yaw():
    return normalize_angle(bno.euler[0])

def align_to_yaw(target):
    print(f"Aligning to {target}° heading...")
    while True:
        yaw = get_current_yaw()
        error = yaw_error(yaw, target)
        print(f"Yaw: {yaw:.1f}°, Error: {error:.1f}°")
        if abs(error) <= YAW_TOLERANCE:
            send_velocity(0, 0, 0, yaw_rate=0)
            print("Yaw aligned.")
            break
        yaw_rate = YAW_RATE if error > 0 else -YAW_RATE
        send_velocity(0, 0, 0, yaw_rate=yaw_rate)
        time.sleep(0.1)
    send_velocity(0, 0, 0, yaw_rate=0)

# === Forward Movement ===
def move_forward(duration):
    print("Moving forward...")
    send_velocity(FORWARD_SPEED, 0, 0)
    time.sleep(duration)
    send_velocity(0, 0, 0)
    print("Forward movement done.")

# === Disarm ===
def stop_and_disarm():
    send_velocity(0, 0, 0, 0)
    print("Disarming...")
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)
    print("Disarmed.")

# === MAIN AUTONOMOUS MISSION ===
try:
    arm_vehicle()
    dive_to_target_depth(TARGET_DEPTH)
    align_to_yaw(TARGET_YAW)
    move_forward(FORWARD_TIME)
    stop_and_disarm()

except KeyboardInterrupt:
    print("Mission aborted.")
    stop_and_disarm()

finally:
    vehicle.close()
    print("Mission complete.")
