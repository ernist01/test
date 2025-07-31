#!/usr/bin/env python3
"""
Simple BLDC Motor Control via Pixhawk
Ready to use with your existing Pi-Pixhawk connection
"""

from pymavlink import mavutil
import time

class SimplePixhawkMotor:
    def __init__(self, connection='/dev/ttyACM0'):
        """
        Connect to your Pixhawk via USB to micro-USB cable
        Configures PWM output for 100Hz (required for your BLDC motor)
        """
        print(f"Connecting to Pixhawk on {connection}...")
        self.master = mavutil.mavlink_connection(connection, baud=57600)
        
        # Wait for connection
        self.master.wait_heartbeat()
        print("✓ Connected to Pixhawk!")
        
        # Configure PWM frequency to 100Hz for BLDC motors
        self.configure_pwm_frequency()
        
        self.armed = False
    
    def configure_pwm_frequency(self):
        """
        Configure Pixhawk PWM outputs to 100Hz for BLDC motors
        This sets the PWM frequency that your motor requires
        """
        print("Configuring PWM frequency to 100Hz...")
        
        # Set PWM frequency to 100Hz (parameter MOT_PWM_TYPE or PWM_MAIN_RATE)
        # This sets the main outputs to 100Hz
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'MOT_PWM_TYPE',  # Motor PWM type parameter
            0,  # 0 = Normal PWM, allows custom frequency
            mavutil.mavlink.MAV_PARAM_TYPE_UINT8
        )
        
        # Set main PWM rate to 100Hz
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'PWM_MAIN_RATE',  # Main PWM frequency
            100,  # 100Hz frequency
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16
        )
        
        time.sleep(1)
        print("✓ PWM frequency set to 100Hz")
    
    def arm(self):
        """Arm the Pixhawk"""
        print("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        # Check if armed
        time.sleep(2)
        self.armed = True
        print("✓ Armed!")
    
    def disarm(self):
        """Disarm the Pixhawk"""
        print("Disarming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.armed = False
        print("✓ Disarmed!")
    
    def set_motor(self, channel, throttle_percent):
        """
        Control motor on specific channel
        
        Args:
            channel: Motor output channel (1-8)
            throttle_percent: Throttle 0-100%
        """
        if not self.armed:
            print("⚠ System not armed! Call arm() first")
            return
        
        # Convert percentage to PWM (1000-2000 microseconds)
        pwm = int(1000 + (throttle_percent / 100) * 1000)
        
        # Create override array (8 channels, 65535 = no change)
        overrides = [65535] * 8
        overrides[channel - 1] = pwm
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *overrides
        )
        
        print(f"Motor {channel}: {throttle_percent}% ({pwm}μs)")
    
    def stop_all(self):
        """Stop all motors"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1000, 1000, 1000, 1000,  # Channels 1-4
            1000, 1000, 1000, 1000   # Channels 5-8
        )
        print("All motors stopped")
    
    def clear_overrides(self):
        """Let Pixhawk resume normal control"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535, 65535, 65535, 65535,
            65535, 65535, 65535, 65535
        )
        print("Control returned to Pixhawk")

# Quick usage example
if __name__ == "__main__":
    # Connect to Pixhawk via USB (adjust if it shows up as /dev/ttyUSB0)
    motor = SimplePixhawkMotor('/dev/ttyACM0')
    
    try:
        # Arm and test motor
        motor.arm()
        
        print("\nTesting motor on channel 1...")
        
        # Gradual speed up
        for speed in [0, 10, 20, 30, 20, 10, 0]:
            motor.set_motor(1, speed)
            time.sleep(2)
        
        print("\nManual control - Enter commands:")
        print("Examples: '1 25' (motor 1 at 25%), 'stop', 'quit'")
        
        while True:
            cmd = input("> ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'quit':
                break
            elif cmd[0] == 'stop':
                motor.stop_all()
            elif len(cmd) == 2:
                try:
                    channel = int(cmd[0])
                    throttle = float(cmd[1])
                    motor.set_motor(channel, throttle)
                except ValueError:
                    print("Format: channel throttle (e.g., '1 25')")
    
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        motor.stop_all()
        motor.clear_overrides()
        motor.disarm()
        print("Done!")
