# File: interactive_control.py

import serial
import time
import os
import threading
from pynput import keyboard

# --- Configuration ---
CONFIG = {
    "port": "COM9",
    "baud_rate": 115200,
    "microsteps": 16,
    "max_steps": 1000,
    "initial_pos": 500,
    "initial_speed": 100,
}

# --- State Tracking ---
class MotorState:
    """A class to hold the live state of the motor controller."""
    def __init__(self):
        # Target state that we will command
        self.m1_target_pos = CONFIG["initial_pos"]
        self.m2_target_pos = CONFIG["initial_pos"]
        self.speed = CONFIG["initial_speed"]
        self.step_size = 50  # How many steps to move per key press

        # Last known message from Arduino
        self.last_arduino_msg = ""
        self.running = True

# Global state object
state = MotorState()

# --- Serial Communication ---
# This runs in a separate thread to avoid blocking the main UI
def serial_reader(ser, state):
    """Continuously reads from the serial port in the background."""
    while state.running:
        try:
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    state.last_arduino_msg = response
        except serial.SerialException:
            state.last_arduino_msg = "ERR: Serial port disconnected."
            break
        time.sleep(0.05)

def send_command(ser, command):
    """Sends a command to the Arduino."""
    ser.write(command.encode('utf-8'))

def send_move_command(ser, m1, m2, speed):
    """Constructs and sends a standard motor move command."""
    # Clamp position to physical limits as a safety measure
    m1 = max(0, min(m1, CONFIG["max_steps"]))
    m2 = max(0, min(m2, CONFIG["max_steps"]))
    command = f"<{int(m1)},{int(m2)},{int(speed)},{CONFIG['microsteps']}>\n"
    send_command(ser, command)
    # Update the state's target position
    state.m1_target_pos = m1
    state.m2_target_pos = m2

# --- UI / Dashboard ---
def update_display():
    """Clears the screen and draws the dashboard."""
    os.system('cls' if os.name == 'nt' else 'clear')
    print("--- Live Motor Controller ---")
    print(f"  Target Position: M1={state.m1_target_pos:<4} | M2={state.m2_target_pos:<4}")
    print(f"  Speed (steps/s): {state.speed:<4}   | Step Size: {state.step_size:<4}")
    print("-" * 30)
    print("  [A/D] Rotate | [W/S] Speed | [Q/E] Step Size")
    print("  [SPACE] STOP | [R] Reset Pos | [ESC] Quit")
    print("-" * 30)
    print(f"Arduino -> {state.last_arduino_msg}")


# --- Keyboard Input Handling ---
def on_press(key):
    """Handles key press events."""
    try:
        # --- Rotational Movement ---
        if key.char == 'a':  # Rotate left
            new_m1 = state.m1_target_pos + state.step_size
            new_m2 = state.m2_target_pos - state.step_size
            send_move_command(ser, new_m1, new_m2, state.speed)
        elif key.char == 'd':  # Rotate rightaaaaaaaadddddddadd
            new_m1 = state.m1_target_pos - state.step_size
            new_m2 = state.m2_target_pos + state.step_size
            send_move_command(ser, new_m1, new_m2, state.speed)
        
        # --- Speed and Step Size ---
        elif key.char == 'w':
            state.speed += 50
        elif key.char == 's':
            state.speed = max(50, state.speed - 50)
        elif key.char == 'e':
            state.step_size += 10
        elif key.char == 'q':
            state.step_size = max(10, state.step_size - 10)

        # --- Utility Commands ---
        elif key.char == 'r': # Reset to initial position
             send_move_command(ser, CONFIG["initial_pos"], CONFIG["initial_pos"], state.speed)

    except AttributeError:
        # Handle special keys like spacebar and escape
        if key == keyboard.Key.space:
            send_command(ser, "<STOP>\n")
        elif key == keyboard.Key.esc:
            # Stop the listener and the program
            state.running = False
            return False
            
# --- Main Execution ---
if __name__ == "__main__":
    ser = None
    try:
        ser = serial.Serial(CONFIG["port"], CONFIG["baud_rate"], timeout=1)
        print(f"Connected to {CONFIG['port']}. Starting control loop...")
        time.sleep(2) # Give Arduino time to boot

        # Start the background thread for reading serial messages
        reader = threading.Thread(target=serial_reader, args=(ser, state))
        reader.daemon = True # Allows main program to exit even if thread is running
        reader.start()

        # Perform initial homing
        print("Performing initial homing...")
        send_command(ser, "<HOME>\n")
        time.sleep(10) # Give homing time to complete, adjust as needed

        # Move to starting center position
        send_move_command(ser, CONFIG["initial_pos"], CONFIG["initial_pos"], state.speed)

        # Start the keyboard listener
        with keyboard.Listener(on_press=on_press) as listener:
            while state.running:
                update_display()
                time.sleep(0.1) # Refresh rate for the display
            listener.join()
    
    except serial.SerialException as e:
        print(f"\nError: Could not open serial port {CONFIG['port']}.")
        print(f"Details: {e}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("\nExiting program...")
        if ser and ser.is_open:
            try:
                # Send a final stop and reset command
                print("Sending reset command...")
                send_command(ser, "<STOP>\n")
                time.sleep(0.1)
                send_move_command(ser, 0, 0, 500)
                time.sleep(2)
            finally:
                state.running = False
                ser.close()
                print("Serial port closed.")