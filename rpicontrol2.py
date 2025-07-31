import RPi.GPIO as GPIO
import time

def setup_pwm(pin, frequency=50):
    """
    Set up PWM on the specified GPIO pin at the given frequency.
    Returns the PWM object for further control.
    """
    GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
    GPIO.setup(pin, GPIO.OUT)  # Set pin as output
    pwm = GPIO.PWM(pin, frequency)  # Create PWM object
    pwm.start(0)  # Start PWM with 0% duty cycle (motor off initially)
    return pwm

def gradually_increase_speed(pwm, max_duty=100, step=5, delay=0.5):
    """
    Gradually increase the duty cycle to spin the motor up smoothly.
    Args:
        pwm: PWM object.
        max_duty: Maximum duty cycle (motor full speed).
        step: Increment to increase duty cycle.
        delay: Delay time between steps (in seconds).
    """
    try:
        print("Starting motor spin-up...")
        for duty_cycle in range(0, max_duty + 1, step):
            pwm.ChangeDutyCycle(duty_cycle)  # Set new duty cycle
            print(f"Motor speed: {duty_cycle}%")
            time.sleep(delay)  # Wait for a smooth transition
        print("Motor at full speed.")
    except KeyboardInterrupt:
        print("\nMotor spin-up interrupted.")
        pwm.stop()
        GPIO.cleanup()

def stop_motor(pwm):
    """
    Stop the motor by setting the duty cycle to 0 and cleaning up GPIO.
    """
    print("Stopping motor...")
    pwm.ChangeDutyCycle(0)  # Stop the motor (0% duty cycle)
    time.sleep(1)
    pwm.stop()
    GPIO.cleanup()
    print("Motor stopped.")

if __name__ == "__main__":
    PWM_PIN = 18  # Pin 18 (GPIO18) for PWM output
    try:
        pwm = setup_pwm(PWM_PIN, 50)  # Set up PWM on GPIO18 with 50Hz frequency
        
        # Gradually increase the motor speed (smooth ramp-up)
        gradually_increase_speed(pwm, max_duty=100, step=10, delay=1)
        
        # Run the motor at full speed for 5 seconds
        time.sleep(5)
        
        # Stop the motor gracefully
        stop_motor(pwm)
        
    except Exception as e:
        print(f"An error occurred: {e}")
        GPIO.cleanup()
