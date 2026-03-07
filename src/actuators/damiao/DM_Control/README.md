# Daimiao Python Library

This library supports Linux and Windows. Example code has been tested on Windows.

Welcome to join QQ group: 677900232 for Daimiao motor technical discussions.

## Table of Contents
- [Installation](#installation)
- [Basic Usage](#basic-usage)
- [Motor Control Modes](#motor-control-modes)
- [Example Scripts](#example-scripts)
- [API Reference](#api-reference)

## Installation

### Dependencies
The motor library requires the following Python packages:
```bash
pip install serial numpy struct
```

### Import the Library
```python
from DM_CAN import *
```

## Basic Usage

### 1. Define Motor Objects
Define motor objects for each motor you want to control. **Important**: Do not set MasterID to 0x00.

```python
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x13)
```

Parameters:
- **MotorType**: Motor type (e.g., `DM_Motor_Type.DM4310`)
- **SlaveID**: CAN ID of the motor (motor ID)
- **MasterID**: Host ID (recommended to be different from SlaveID, typically higher)

### 2. Setup Serial Connection
Configure the serial port with baud rate 921600:

```python
import serial
serial_device = serial.Serial('COM3', 921600, timeout=0.5)  # Windows
# serial_device = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.5)  # Linux
```

### 3. Initialize Motor Controller
```python
MotorControl1 = MotorControl(serial_device)
```

### 4. Add and Enable Motors
```python
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)

MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
```

**Note**: It's recommended to add a 1-2ms delay after each control frame for USB-to-CAN adapters with buffers.

## Motor Control Modes

### 1. MIT Mode (Recommended)
MIT mode provides the most flexible control with position, velocity, and torque parameters.

```python
# Basic MIT control
MotorControl1.controlMIT(Motor1, kp=50, kd=0.3, q=0, dq=0, tau=0)

# MIT control with delay
MotorControl1.control_delay(Motor1, kp=50, kd=0.3, q=0, dq=0, tau=0, delay=0.001)
```

Parameters:
- `kp`: Position gain
- `kd`: Velocity gain  
- `q`: Desired position (rad)
- `dq`: Desired velocity (rad/s)
- `tau`: Desired torque (Nm)

### 2. Position-Velocity Mode
Control motor position with specified velocity.

```python
import math
import time

q = math.sin(time.time())
MotorControl1.control_Pos_Vel(Motor1, q * 10, 2)
```

### 3. Velocity Mode
Direct velocity control.

```python
q = math.sin(time.time())
MotorControl1.control_Vel(Motor1, q * 5)
```

### 4. Position-Force Mode (EMIT)
Hybrid position and force control.

```python
MotorControl1.control_pos_force(Motor1, position=10, velocity=1000, current=100)
```

Parameters:
- `position`: Desired position (rad)
- `velocity`: Velocity limit (0-10000, scaled by 100)
- `current`: Current limit (0-10000, normalized current value)

## Example Scripts

### 1. Zero Position Setting (`zero_set_motor.py`)
Simple script to set the motor's zero position:

```python
from DM_CAN import *
import serial
import time

Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
serial_device = serial.Serial('COM3', 921600, timeout=0.5)
Motor_Control = MotorControl(serial_device)
Motor_Control.addMotor(Motor1)
Motor_Control.set_zero_position(Motor1)
time.sleep(0.2)
print("Set Zero Position Successfully!")
serial_device.close()
```

### 2. Gripper Control (`test_gripper.py`)
Interactive gripper control with safety limits and percentage-based control:

```bash
python test_gripper.py
```

Features:
- **Safety limits**: Position constrained to [-1.35, 0] radians
- **Percentage control**: 0% = fully open, 100% = fully closed
- **Interactive commands**:
  - `o`: Open gripper (0% closed)
  - `c`: Close gripper (100% closed)  
  - `h`: Half-open (50% closed)
  - `1-9, 0`: Move to specific percentage (10%-100%)
  - `q`: Quit

## Motor Mode Switching

The new Daimiao firmware supports online mode switching via CAN commands.

### Available Control Modes
```python
Control_Type.MIT        # MIT control mode
Control_Type.POS_VEL    # Position-velocity mode
Control_Type.VEL        # Velocity mode  
Control_Type.Torque_Pos # Torque-position mode
```

### Switch Control Mode
```python
MotorControl1.switchControlMode(Motor1, Control_Type.POS_VEL)
```

### Save Parameters to Flash
By default, parameter changes are not saved to flash memory. Use this command to save:

```python
MotorControl1.save_motor_param(Motor1)
```

## Supported Motor Types

```python
DM_Motor_Type.DM4310      # DM4310 motor
DM_Motor_Type.DM4310_48V  # DM4310 48V motor
DM_Motor_Type.DM4340      # DM4340 motor
DM_Motor_Type.DM4340_48V  # DM4340 48V motor
DM_Motor_Type.DM6006      # DM6006 motor
DM_Motor_Type.DM8006      # DM8006 motor
DM_Motor_Type.DM8009      # DM8009 motor
DM_Motor_Type.DM10010L    # DM10010L motor
DM_Motor_Type.DM10010     # DM10010 motor
DM_Motor_Type.DMH3510     # DMH3510 motor
DM_Motor_Type.DMH6215     # DMH6215 motor
DM_Motor_Type.DMG6220     # DMG6220 motor
```

## API Reference

### Motor Class
```python
motor = Motor(MotorType, SlaveID, MasterID)
motor.getPosition()    # Get current position
motor.getVelocity()    # Get current velocity  
motor.getTorque()      # Get current torque
```

### MotorControl Class
```python
controller = MotorControl(serial_device)
controller.addMotor(motor)                    # Add motor to controller
controller.enable(motor)                      # Enable motor
controller.disable(motor)                     # Disable motor
controller.set_zero_position(motor)           # Set zero position
controller.switchControlMode(motor, mode)     # Switch control mode
controller.save_motor_param(motor)            # Save parameters to flash
```

### Control Methods
```python
controller.controlMIT(motor, kp, kd, q, dq, tau)           # MIT control
controller.control_Pos_Vel(motor, position, velocity)      # Position-velocity control
controller.control_Vel(motor, velocity)                    # Velocity control
controller.control_pos_force(motor, pos, vel, current)     # Position-force control
```

## Troubleshooting

1. **Serial Port Issues**: Ensure correct COM port and baud rate (921600)
2. **Motor Not Found**: Verify SlaveID and MasterID settings
3. **Control Mode Errors**: Check if motor is enabled before switching modes
4. **Parameter Changes**: Remember to save parameters to flash if needed

For detailed technical documentation, please refer to the Daimiao motor documentation or contact Daimiao customer service.

