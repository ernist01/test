#!/usr/bin/env python3
"""
Control BLDC motor via Pixhawk using MAVLink from Raspberry Pi
This approach is much more reliable than direct PWM control
"""

from pymavlink import mavutil
import time
import sys

class PixhawkBLDCController:
    def __init__(self, connection_string='/dev/ttyUSB0', baudrate=57600):
        """
        Initialize Pixhawk connection for BLDC motor control
        
        Args:
            connection_string: Serial port or network connection
            baudrate: Communication baudrate
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.system_id = 1
        self.component_id = 1
        
        # Motor channel mapping (adjust based on your setup)
        self.motor_channels = {
            'motor1': 1,  # Main output 1
            'motor2': 2,  # Main output 2
            'motor3': 3,  # Main output 3
            'motor4': 4,  # Main output 4
        }
        
        self.connect()
    
    def connect(self):
        """Establish connection to Pixhawk"""
        try:
            print(f"Connecting to Pixhawk on {self.connection_string}...")
            self.master = mavutil.mavlink_connection(
                self.connection_string, 
                baud=self.baudrate
            )
            
            # Wait for heartbeat
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")
            
            # Request data streams
            self.request_data_streams()
            
        except Exception as e:
            print(f"Connection failed: {e}")
            sys.exit(1)
    
    def request_data_streams(self):
        """Request necessary data streams from Pixhawk"""
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # 4Hz
            1   # Enable
        )
    
    def arm(self):
        """Arm the Pixhawk (required before motor control)"""
        print("Arming Pixhawk...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Armed successfully!")
            return True
        else:
            print("Arming failed!")
            return False
    
    def disarm(self):
        """Disarm the Pixhawk"""
        print("Disarming Pixhawk...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Disarmed successfully!")
            return True
        else:
            print("Disarming failed!")
            return False
    
    def set_motor_pwm(self, channel, pwm_value):
        """
        Set PWM value for specific motor channel
        
        Args:
            channel: Motor channel (1-8)
            pwm_value: PWM value in microseconds (1000-2000)
        """
        if not 1000 <= pwm_value <= 2000:
            print("Error: PWM value must be between 1000-2000 microseconds")
            return
        
        if not 1 <= channel <= 8:
            print("Error: Channel must be between 1-8")
            return
        
        # Create PWM values array (8 channels)
        pwm_values = [65535] * 8  # 65535 = ignore/no change
        pwm_values[channel - 1] = pwm_value  # Set specific channel
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *pwm_values
        )
        
        print(f"Channel {channel}: {pwm_value} μs")
    
    def set_motor_throttle_percent(self, channel, throttle_percent):
        """
        Set motor throttle as percentage
        
        Args:
            channel: Motor channel (1-8)
            throttle_percent: Throttle percentage (0-100)
        """
        if not 0 <= throttle_percent <= 100:
            print("Error: Throttle must be between 0-100%")
            return
        
        # Convert percentage to PWM microseconds
        pwm_value = int(1000 + (throttle_percent / 100) * 1000)
        self.set_motor_pwm(channel, pwm_value)
    
    def set_all_motors(self, pwm_value):
        """Set all motors to same PWM value"""
        pwm_values = [pwm_value] * 8
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *pwm_values
        )
        
        print(f"All motors: {pwm_value} μs")
    
    def stop_all_motors(self):
        """Stop all motors (minimum throttle)"""
        self.set_all_motors(1000)
        print("All motors stopped")
    
    def clear_overrides(self):
        """Clear RC channel overrides"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535, 65535, 65535, 65535,  # Channels 1-4
            65535, 65535, 65535, 65535   # Channels 5-8
        )
        print("RC overrides cleared")
    
    def get_battery_status(self):
        """Get battery voltage and current"""
        msg = self.master.recv_match(type='BATTERY_STATUS', blocking=False)
        if msg:
            voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            return voltage, current
        return None, None
    
    def monitor_system(self):
        """Monitor system status"""
        voltage, current = self.get_battery_status()
        if voltage and current:
            print(f"Battery: {voltage:.2f}V, {current:.2f}A")

def main():
    """Main function for motor control demo"""
    
    # Connection options:
    # Serial: '/dev/ttyUSB0' or '/dev/ttyACM0'
    # Network: 'udp:127.0.0.1:14550'
    controller = PixhawkBLDCController('/dev/ttyUSB0', 57600)
    
    try:
        # Arm the system
        if not controller.arm():
            print("Failed to arm, exiting...")
            return
        
        print("\nStarting motor control demo...")
        print("Motor will be controlled on channel 1")
        print("Press Ctrl+C to stop")
        
        time.sleep(2)
        
        # Gradual speed test
        print("\nGradual speed increase...")
        for throttle in range(0, 51, 10):  # 0% to 50%
            controller.set_motor_throttle_percent(1, throttle)
            controller.monitor_system()
            time.sleep(3)
        
        print("\nHolding at 50% throttle...")
        time.sleep(5)
        
        print("\nGradual speed decrease...")
        for throttle in range(50, -1, -10):  # 50% to 0%
            controller.set_motor_throttle_percent(1, throttle)
            controller.monitor_system()
            time.sleep(2)
        
        # Manual control mode
        print("\nEntering manual control mode...")
        print("Commands:")
        print("  't <channel> <percent>' - Set throttle (e.g., 't 1 25')")
        print("  'p <channel> <pwm>' - Set PWM directly (e.g., 'p 1 1500')")
        print("  's' - Stop all motors")
        print("  'q' - Quit")
        
        while True:
            try:
                cmd = input("\nCommand: ").strip().split()
                
                if not cmd:
                    continue
                
                if cmd[0] == 'q':
                    break
                elif cmd[0] == 's':
                    controller.stop_all_motors()
                elif cmd[0] == 't' and len(cmd) == 3:
                    channel = int(cmd[1])
                    percent = float(cmd[2])
                    controller.set_motor_throttle_percent(channel, percent)
                elif cmd[0] == 'p' and len(cmd) == 3:
                    channel = int(cmd[1])
                    pwm = int(cmd[2])
                    controller.set_motor_pwm(channel, pwm)
                else:
                    print("Invalid command")
                
                controller.monitor_system()
                
            except (ValueError, IndexError):
                print("Invalid input format")
            except KeyboardInterrupt:
                break
    
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        # Safety: stop all motors and disarm
        controller.stop_all_motors()
        time.sleep(1)
        controller.clear_overrides()
        controller.disarm()
        print("Motor control stopped safely")

if __name__ == "__main__":
    main()
