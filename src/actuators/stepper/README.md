# Stepper Motor Control System

This directory contains a complete stepper motor control system for dual-motor applications, featuring a Python-based interactive controller and Arduino firmware for motor control.

## Overview

The system consists of two main components:
- **Python Controller** (`stepper_control.py`): Interactive keyboard-based control interface
- **Arduino Firmware** (`stepper_remote/stepper_remote.ino`): Low-level motor control and safety management

## Features

- **Dual Motor Control**: Simultaneous control of two stepper motors
- **Interactive Interface**: Real-time keyboard control with live status display
- **Safety Features**: 
  - Power monitoring and automatic shutdown
  - Position limits and emergency stop
  - Motor enable/disable control
- **Homing Sequence**: Automatic homing on startup
- **Non-blocking Operation**: Smooth motor movement without blocking the main loop
- **Configurable Parameters**: Adjustable speed, step size, and microstepping

## Hardware Requirements

### Arduino Setup
- Arduino board (compatible with standard Arduino IDE)
- Dual path stepper motor drivers
- 2x Stepper motors
- 12V power supply
- Voltage divider circuit for power monitoring (ADC pin A0)

### Pin Connections
```
Motor 1:
- Enable: Pin 7
- Direction: Pin 8  
- Step: Pin 9

Motor 2:
- Enable: Pin 5
- Direction: Pin 4
- Step: Pin 3

Power Monitoring:
- ADC Input: Pin A0 (voltage divider from 12V supply)
```

## Software Requirements

### Python Dependencies
```bash
pip install pyserial pynput
```

### Arduino Dependencies
- Standard Arduino IDE
- No additional libraries required

## Setup Instructions

### 1. Arduino Firmware Setup
1. Open `stepper_remote/stepper_remote.ino` in Arduino IDE
2. Connect Arduino to your computer
3. Select the correct board and port
4. Upload the firmware
5. Open Serial Monitor (115200 baud) to verify communication

### 2. Python Controller Setup
1. Install required Python packages:
   ```bash
   pip install pyserial pynput
   ```
2. Modify the configuration in `stepper_control.py`:
   ```python
   CONFIG = {
       "port": "COM8",        # Change to your Arduino port
       "baud_rate": 115200,
       "microsteps": 16,
       "max_steps": 1000,
       "initial_pos": 500,
       "initial_speed": 100,
   }
   ```

## Usage

### Starting the System
1. Ensure Arduino is connected and firmware is uploaded
2. Connect 12V power supply
3. Run the Python controller:
   ```bash
   python stepper_control.py
   ```

### Control Interface
The system provides a live dashboard with the following controls:

- **Movement Controls**:
  - `A` - Rotate left
  - `D` - Rotate right
  
- **Speed and Step Size**:
  - `W` - Increase speed (+50 steps/sec)
  - `S` - Decrease speed (-50 steps/sec)
  - `E` - Increase step size (+10 steps)
  - `Q` - Decrease step size (-10 steps)
  
- **Utility Commands**:
  - `SPACE` - Emergency stop
  - `R` - Reset to initial position
  - `ESC` - Quit program

### Command Protocol
The system uses a simple serial protocol for communication:

- **Move Command**: `<m1_pos,m2_pos,speed,microsteps>`
  - Example: `<500,500,100,16>` - Move both motors to position 500 at 100 steps/sec
- **Stop Command**: `<STOP>` - Emergency stop
- **Home Command**: `<HOME>` - Initiate homing sequence

## Configuration

### Motor Parameters
- **Microsteps**: 16 (configurable)
- **Full Travel**: 1000 steps (physical limit)
- **Homing Speed**: 8000 Hz
- **Voltage Threshold**: 5.0V minimum

### Safety Features
- **Position Limits**: 0-1000 steps per motor
- **Power Monitoring**: Automatic shutdown if 12V power is lost
- **Motor Enable Control**: Motors are disabled when not moving
- **Emergency Stop**: Immediate halt and motor disable

## Troubleshooting

### Common Issues

1. **Serial Port Not Found**
   - Check Arduino port in Device Manager (Windows) or `/dev/tty*` (Linux/Mac)
   - Update `CONFIG["port"]` in `stepper_control.py`

2. **Motors Not Moving**
   - Verify 12V power supply is connected
   - Check motor driver connections
   - Ensure motors are enabled (check enable pins)

3. **Communication Errors**
   - Verify baud rate matches (115200)
   - Check USB connection
   - Restart Arduino and Python program

4. **Homing Issues**
   - Ensure motors can reach both end positions
   - Check for mechanical obstructions
   - Verify motor wiring

### Debug Information
The Python controller displays real-time status including:
- Current target positions
- Speed and step size settings
- Arduino communication status
- Error messages from Arduino

## File Structure
```
stepper/
├── README.md                 # This file
├── stepper_control.py        # Python controller
└── stepper_remote/
    └── stepper_remote.ino    # Arduino firmware
```

## Safety Notes

- Always ensure proper power supply before operation
- Keep emergency stop accessible during testing
- Verify mechanical limits before running homing sequence
- Monitor motor temperature during extended operation
- Disconnect power when making wiring changes

## License

This project is part of the Multimodal Intervention Robot system. Please refer to the main project license for usage terms. 