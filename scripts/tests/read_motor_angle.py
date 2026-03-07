import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'actuators', 'dynamixel'))

from dynamixel_sdk import *  # Import the SDK
from libgx.motor import Motor

# Initialize PortHandler and PacketHandler
portHandler = PortHandler('COM4')  # Replace with your port
packetHandler = PacketHandler(2.0)  # Protocol version
BAUD_RATE = 1000000

# Open port
if not portHandler.openPort():
    print("Failed to open port")
    quit()

# Set baudrate
if not portHandler.setBaudRate(BAUD_RATE):
    print("Failed to set baudrate")
    quit()

# Initialize Motor instance
DXL_ID = 7  # Motor ID
motor = Motor(DXL_ID, portHandler, packetHandler)

# Set motor to position operating mode
motor.torq_off()
motor.packet_handler.write1ByteTxRx(portHandler, DXL_ID, motor.addr_operating_mode, motor.pos_operating_mode)

try:
    print("Reading motor angle. Press Ctrl+C to stop.")
    while True:
        # Get motor position in degrees
        angle_in_degrees = motor.get_pos()
        # Overwrite the previous line in the terminal
        sys.stdout.write(f"\rCurrent Angle: {angle_in_degrees:.2f} degrees")
        sys.stdout.flush()

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    # Close port
    portHandler.closePort()
