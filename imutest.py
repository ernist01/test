import board
import busio
import adafruit_bno08x
import time

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

try:
    # Initialize BNO085 sensor
    bno = adafruit_bno08x.BNO08X_I2C(i2c)

    # Read temperature as a test
    print("Temperature: ", bno.temperature)

except Exception as e:
    print("Error:", e)
