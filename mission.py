import time
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

from dronekit import connect, VehicleMode, mavutil

# === CONFIGURATION ===
TARGET_DEPTH = 1.5       # Meters (not used for feedback, just timed)
DEPTH_TIME = 5           # Seconds to descend
FORWARD_TIME = 10        # Seconds to go forward
DESCEND_SPEED = 0.3      # m/s (positive = down in NED)
FORWARD_SPEED = 0.4      # m/s
YAW_TOLERANCE = 5.0      # Degrees
YAW_RATE = 0.3           # rad/s
TARGET_YAW = 90.0        # Desired heading in degrees (East)

# === BNO085 SETUP ===
print("Setting up IMU...")
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

# === Pixhawk SETUP ===
print("Connecting to Pixhawk...")
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

# === MAVLINK SEND FUNCTION ===
def send_velocity(vx, vy, vz, yaw_rate=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Enable velocity and yaw_rate
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

# === CONTROL ===
def arm_vehicle():
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(" Waiting for GUIDED mode...")
        time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Vehicle armed.")

def disarm_vehicle():
    vehicle.armed = False
    while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(1)
    print("Vehicle disarmed.")

def stop_all():
    send_velocity(0, 0, 0)

def align_heading(target_yaw, tolerance=YAW_TOLERANCE):
    print(f"Aligning heading to {target_yaw}°...")
    while True:
        yaw = normalize_angle(bno.euler[0])
        error = yaw_error(yaw, target_yaw)
        print(f"Current yaw: {yaw:.1f}°, error: {error:.1f}°")

        if abs(error) < tolerance:
            print("Heading aligned.")
            send_velocity(0, 0, 0)
            break

        if error > 0:
            send_velocity(0, 0, 0, yaw_rate=YAW_RATE)
        else:
            send_velocity(0, 0, 0, yaw_rate=-YAW_RATE)

        time.sleep(0.1)

# === MAIN MISSION ===
try:
    print("=== Starting Autonomous Mission ===")
    arm_vehicle()
    time.sleep(2)

    # Step 1: Dive to depth (time-based)
    print(f"Diving for {DEPTH_TIME} seconds...")
    send_velocity(0, 0, DESCEND_SPEED)
    time.sleep(DEPTH_TIME)
    stop_all()
    time.sleep(1)

    # Step 2: Align heading
    align_heading(TARGET_YAW)
    time.sleep(1)

    # Step 3: Move forward
    print(f"Moving forward for {FORWARD_TIME} seconds...")
    send_velocity(FORWARD_SPEED, 0, 0)
    time.sleep(FORWARD_TIME)
    stop_all()

    # Step 4: Disarm and finish
    print("Mission complete.")
    disarm_vehicle()

except KeyboardInterrupt:
    print("Mission aborted!")
    stop_all()
    disarm_vehicle()

finally:
    vehicle.close()
    print("Disconnected.")
