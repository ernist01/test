#!/usr/bin/env python3
"""
BLDC Motor Control with ESC using Raspberry Pi 4
PWM Frequency: 100Hz (required for your ESC)
"""

import RPi.GPIO as GPIO
import time

class BLDCController:
    def __init__(self, pwm_pin=18):
        """
        Initialize BLDC motor controller
        
        Args:
            pwm_pin: GPIO pin for PWM signal (default: 18)
        """
        self.pwm_pin = pwm_pin
        self.pwm = None
        
        # PWM settings
        self.frequency = 100  # 100Hz as required by your ESC
        
        # Typical ESC pulse width values (in milliseconds)
        self.min_pulse_width = 1.0   # 1ms - minimum throttle
        self.max_pulse_width = 2.0   # 2ms - maximum throttle
        self.neutral_pulse_width = 1.5  # 1.5ms - neutral/stop
        
        # Calculate duty cycles for 100Hz (10ms period)
        self.period_ms = 1000 / self.frequency  # 10ms for 100Hz
        self.min_duty = (self.min_pulse_width / self.period_ms) * 100
        self.max_duty = (self.max_pulse_width / self.period_ms) * 100
        self.neutral_duty = (self.neutral_pulse_width / self.period_ms) * 100
        
        self.setup_gpio()
    
    def setup_gpio(self):
        """Setup GPIO and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        
        # Create PWM instance with 100Hz frequency
        self.pwm = GPIO.PWM(self.pwm_pin, self.frequency)
        
        print(f"PWM setup: {self.frequency}Hz on GPIO {self.pwm_pin}")
        print(f"Duty cycle range: {self.min_duty:.1f}% to {self.max_duty:.1f}%")
    
    def arm_esc(self):
        """
        Arm the ESC by sending minimum throttle signal
        This is typically required before the ESC will respond to throttle commands
        """
        print("Arming ESC...")
        self.pwm.start(self.min_duty)
        print(f"Sending minimum throttle signal ({self.min_duty:.1f}% duty cycle)")
        time.sleep(2)  # Wait for ESC to arm
        print("ESC armed!")
    
    def set_throttle_percent(self, throttle_percent):
        """
        Set motor throttle as percentage (0-100%)
        
        Args:
            throttle_percent: Throttle percentage (0 = min, 100 = max)
        """
        if not 0 <= throttle_percent <= 100:
            print("Error: Throttle must be between 0 and 100%")
            return
        
        # Calculate duty cycle based on throttle percentage
        duty_range = self.max_duty - self.min_duty
        target_duty = self.min_duty + (throttle_percent / 100) * duty_range
        
        self.pwm.ChangeDutyCycle(target_duty)
        print(f"Throttle: {throttle_percent}% (Duty: {target_duty:.1f}%)")
    
    def set_throttle_raw(self, pulse_width_ms):
        """
        Set motor throttle using raw pulse width in milliseconds
        
        Args:
            pulse_width_ms: Pulse width in milliseconds (1.0 - 2.0)
        """
        if not self.min_pulse_width <= pulse_width_ms <= self.max_pulse_width:
            print(f"Error: Pulse width must be between {self.min_pulse_width} and {self.max_pulse_width} ms")
            return
        
        duty_cycle = (pulse_width_ms / self.period_ms) * 100
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"Pulse width: {pulse_width_ms}ms (Duty: {duty_cycle:.1f}%)")
    
    def stop_motor(self):
        """Stop the motor by setting throttle to minimum"""
        self.set_throttle_percent(0)
        print("Motor stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.pwm:
            self.pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

def main():
    """Main function demonstrating motor control"""
    
    # Create controller instance
    controller = BLDCController(pwm_pin=18)  # Use GPIO 18 for PWM
    
    try:
        # Arm the ESC
        controller.arm_esc()
        
        print("\nStarting motor control demo...")
        print("Press Ctrl+C to stop")
        
        # Gradual speed increase
        print("\nGradually increasing speed...")
        for throttle in range(0, 51, 10):  # 0% to 50% in 10% steps
            controller.set_throttle_percent(throttle)
            time.sleep(2)
        
        # Hold at 50% for a moment
        print("Holding at 50% throttle...")
        time.sleep(3)
        
        # Gradual speed decrease
        print("\nGradually decreasing speed...")
        for throttle in range(50, -1, -10):  # 50% to 0% in 10% steps
            controller.set_throttle_percent(throttle)
            time.sleep(2)
        
        # Alternative: Manual control loop
        print("\nEntering manual control mode...")
        print("Enter throttle percentage (0-100) or 'q' to quit:")
        
        while True:
            user_input = input("Throttle %: ").strip()
            
            if user_input.lower() == 'q':
                break
            
            try:
                throttle = float(user_input)
                controller.set_throttle_percent(throttle)
            except ValueError:
                print("Please enter a valid number or 'q' to quit")
    
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        # Always stop motor and cleanup
        controller.stop_motor()
        time.sleep(1)
        controller.cleanup()

if __name__ == "__main__":
    main()
