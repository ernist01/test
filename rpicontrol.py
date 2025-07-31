import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # or GPIO.BOARD for physical pin numbering

# Set GPIO Pin for PWM signal
PWM_PIN = 18  # Pin number for PWM output (GPIO18, for example)

# Set the GPIO pin as an output pin
GPIO.setup(PWM_PIN, GPIO.OUT)

# Set the PWM frequency to 100Hz (for your motor)
pwm = GPIO.PWM(PWM_PIN, 100)  # Frequency of 100Hz

# Start PWM with an initial duty cycle (e.g., 50%)
pwm.start(50)  # Duty cycle is percentage of time PWM is on

# Control motor for 5 seconds (for example)
time.sleep(5)

# Stop PWM signal and clean up GPIO
pwm.stop()
GPIO.cleanup()
