#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
DM Gripper Control
-----------------
Interactive control for a gripper actuated by DM motor with safety position limits.
Uses intuitive percentage-based control where:
- 0% = Gripper fully open (0 rad)
- 100% = Gripper fully closed (-1.35 rad)

Commands:
- 'o': Open gripper fully (0% closed)
- 'c': Close gripper fully (100% closed)
- 'h': Half-open position (50% closed)
- 'p': Move to specific percentage closure (0-100%)
- 'r': Move to specific radian position (advanced, [-1.35, 0] rad)
- 'q': Quit program
"""

import math
import time
import serial
import numpy as np
import msvcrt
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'actuators', 'damiao', 'DM_Control'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

from DM_CAN import *
from src.core.config_loader import GRIPPER_CONFIG, GRIPPER_MIN_POS, GRIPPER_MAX_POS, ADAPTIVE_GRIPPING_CONFIG

# Gripper position limits — sourced from configs/gripper.yaml
DEFAULT_VELOCITY = ADAPTIVE_GRIPPING_CONFIG["opening_velocity"]

def percentage_to_position(percentage):
    """
    Convert closure percentage to radian position
    
    Args:
        percentage (float): Gripper closure percentage (0% = open, 100% = closed)
        
    Returns:
        float: Corresponding position in radians
    """
    # Ensure percentage is within [0, 100]
    percentage = max(0, min(100, percentage))
    
    # Map 0-100% to [GRIPPER_MAX_POS, GRIPPER_MIN_POS]
    # 0% = GRIPPER_MAX_POS (fully open)
    # 100% = GRIPPER_MIN_POS (fully closed)
    position = GRIPPER_MAX_POS + (percentage/100.0) * (GRIPPER_MIN_POS - GRIPPER_MAX_POS)
    
    return position

def position_to_percentage(position):
    """
    Convert radian position to closure percentage
    
    Args:
        position (float): Position in radians
        
    Returns:
        float: Gripper closure percentage (0% = open, 100% = closed)
    """
    # Ensure position is within limits
    position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    
    # Map position from [GRIPPER_MAX_POS, GRIPPER_MIN_POS] to [0, 100]
    percentage = ((position - GRIPPER_MAX_POS) / (GRIPPER_MIN_POS - GRIPPER_MAX_POS)) * 100
    
    return percentage

def safe_gripper_position(position):
    """
    Ensures the gripper position stays within safe limits.
    
    Args:
        position (float): Desired position in radians
        
    Returns:
        float: Position limited to safe range [-1.35, 0]
    """
    limited_position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    if limited_position != position:
        print(f"Position limited: {position:.3f} → {limited_position:.3f}")
    return limited_position

def setup_motor():
    """Initialize and configure the motor"""
    try:
        # Setup motor
        motor = Motor(DM_Motor_Type.DM4310,
                      GRIPPER_CONFIG["motor"]["motor_id"],
                      GRIPPER_CONFIG["motor"]["can_id"])

        # Setup serial connection
        serial_port = serial.Serial(
            GRIPPER_CONFIG["motor"]["port"],
            GRIPPER_CONFIG["motor"]["baud_rate"],
            timeout=0.5,
        )
        motor_control = MotorControl(serial_port)
        
        # Add and enable motor
        motor_control.addMotor(motor)
        motor_control.enable(motor)
        print("✓ Motor enabled successfully")
        
        # Set control mode
        motor_control.switchControlMode(motor, Control_Type.POS_VEL)
        
        return motor, motor_control, serial_port
    
    except Exception as e:
        print(f"❌ Error setting up motor: {e}")
        raise

def move_gripper_to_position(motor_control, motor, position, velocity=DEFAULT_VELOCITY):
    """
    Move gripper to a specific radian position while enforcing safety limits
    
    Args:
        motor_control: Motor controller instance
        motor: Motor instance
        position (float): Target position in radians
        velocity (float): Movement velocity
    """
    safe_pos = safe_gripper_position(position)
    percentage = position_to_percentage(safe_pos)
    print(f"Moving to position: {safe_pos:.3f} rad ({percentage:.1f}% closed) (velocity: {velocity})")
    motor_control.control_Pos_Vel(motor, safe_pos, velocity)
    
def move_gripper(motor_control, motor, percentage, velocity=DEFAULT_VELOCITY):
    """
    Move gripper to a percentage closure while enforcing safety limits
    
    Args:
        motor_control: Motor controller instance
        motor: Motor instance
        percentage (float): Target closure percentage (0% = open, 100% = closed)
        velocity (float): Movement velocity
    """
    # Convert percentage to radians
    position = percentage_to_position(percentage)
    
    # Move to the position
    move_gripper_to_position(motor_control, motor, position, velocity)
    
def keyboard_control():
    """Run interactive control loop for the gripper"""
    motor, motor_control, serial_port = setup_motor()

    try:
        # Home the gripper by moving to open position (0% closed)
        move_gripper(motor_control, motor, 0)  # 0% closed = fully open
        time.sleep(1)

        print("\n" + "-" * 50)
        print("Gripper Control Interface")
        print("-" * 50)
        print("Closure range: 0% (fully open) to 100% (fully closed)")
        print(f"Position range: [{GRIPPER_MAX_POS:.2f}, {GRIPPER_MIN_POS:.2f}] rad")
        print("-" * 50)
        print("Commands: ")
        print("  'o': Open gripper fully (0% closed)")
        print("  'c': Close gripper fully (100% closed)")
        print("  'h': Half-open position (50% closed)")
        print("  Number keys (1-9, 0): Move to closure percentage (10%-100%)")
        print("  'q': Quit")
        print("-" * 50 + "\n")

        while True:
            if msvcrt.kbhit():  # Check if a key is pressed
                cmd = msvcrt.getch().decode('utf-8').strip().lower()

                if cmd == 'q':
                    break
                elif cmd == 'o':
                    move_gripper(motor_control, motor, 0)  # 0% closed
                elif cmd == 'c':
                    move_gripper(motor_control, motor, 100)  # 100% closed
                elif cmd == 'h':
                    move_gripper(motor_control, motor, 50)  # 50% closed
                elif cmd.isdigit():
                    percentage = int(cmd) * 10
                    if percentage == 0:
                        percentage = 100  # Handle '0' as 100%
                    move_gripper(motor_control, motor, percentage)
                else:
                    print("❌ Unknown command. Try again.")

    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt received. Shutting down...")
    finally:
        # Clean shutdown
        print("\nDisabling motor...")
        motor_control.disable(motor)
        print("Closing serial port...")
        serial_port.close()
        print("✓ Cleanup completed")

if __name__ == "__main__":
    keyboard_control()
