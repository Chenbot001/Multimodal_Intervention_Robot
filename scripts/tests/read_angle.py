import minimalmodbus
import time
import keyboard
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import os

# --- Configuration ---
PORT_NAME = 'COM11'
SLAVE_ADDRESS = 1
BAUDRATE = 9600
READ_REGISTER_ADDRESS = 0
ZERO_REGISTER_ADDRESS = 8      # Register for the zero command
DIRECTION_REGISTER_ADDRESS = 9 # Register for the direction command

# Constants for decoding
SINGLE_TURN_RESOLUTION = 2**15

# Data collection variables
is_recording = False
recorded_data = []
recording_start_time = None

def zero_encoder(instrument):
    """Sends the command to zero the encoder."""
    try:
        print("\nSending zeroing command...")
        # Command: 01 06 00 08 00 01 C9 C8
        instrument.write_register(
            registeraddress=ZERO_REGISTER_ADDRESS,
            value=1,
            functioncode=6
        )
        print("Zeroing command sent successfully!")
    except minimalmodbus.ModbusException as e:
        print(f"Failed to send zeroing command: {e}")

def set_direction(instrument, set_to_ccw):
    """
    Sends the command to set the encoder direction.
    - CW:  Write 0 to register 9 (01 06 00 09 00 00 59 C8)
    - CCW: Write 1 to register 9 (01 06 00 09 00 01 98 08)
    """
    try:
        value_to_write = 1 if set_to_ccw else 0
        direction_str = "Counter-Clockwise (CCW)" if set_to_ccw else "Clockwise (CW)"
        
        print(f"\nSetting direction to {direction_str}...")
        instrument.write_register(
            registeraddress=DIRECTION_REGISTER_ADDRESS,
            value=value_to_write,
            functioncode=6
        )
        print("Direction set successfully!")
        return set_to_ccw
    except minimalmodbus.ModbusException as e:
        print(f"Failed to set direction: {e}")
        # If the command fails, return the previous state
        return not set_to_ccw

def start_recording():
    """Start recording data."""
    global is_recording, recorded_data, recording_start_time
    if not is_recording:
        is_recording = True
        recorded_data = []
        recording_start_time = time.time()
        print(f"\n🎙️  Recording started at {datetime.now().strftime('%H:%M:%S')}")

def stop_recording():
    """Stop recording data and save to file."""
    global is_recording, recorded_data, recording_start_time
    if is_recording:
        is_recording = False
        recording_duration = time.time() - recording_start_time
        print(f"\n⏹️  Recording stopped. Duration: {recording_duration:.2f}s")
        print(f"📊 Collected {len(recorded_data)} data points")
        
        if recorded_data:
            save_and_plot_data(recorded_data, recording_duration)
        recorded_data = []
        recording_start_time = None

def unwrap_angles(angles):
    """
    Unwrap angle data to handle 0°/360° transitions smoothly.
    
    Args:
        angles (list): List of angle values in degrees
    
    Returns:
        list: Unwrapped angle values (continuous, may exceed 360°)
    """
    if not angles:
        return angles
    
    unwrapped = [angles[0]]  # Start with the first angle
    
    for i in range(1, len(angles)):
        prev_angle = unwrapped[i-1]
        current_angle = angles[i]
        
        # Calculate the difference
        diff = current_angle - (prev_angle % 360)
        
        # Handle wrap-around cases
        if diff > 180:  # Wrapped from ~360° to ~0°
            adjustment = -360
        elif diff < -180:  # Wrapped from ~0° to ~360°
            adjustment = 360
        else:
            adjustment = 0
        
        # Add the adjusted angle
        unwrapped.append(prev_angle + diff + adjustment)
    
    return unwrapped

def save_and_plot_data(data, duration):
    """Save data to CSV and create plots."""
    if not data:
        print("No data to save/plot")
        return
    
    # Create filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"rotary_encoder/encoder_data_{timestamp}.csv"
    plot_filename = f"rotary_encoder/encoder_plot_{timestamp}.png"
    
    # Save to CSV
    try:
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Time(s)', 'Angle(deg)', 'Turns', 'Single_Turn', 'Raw_Value', 'Direction'])
            writer.writerows(data)
        print(f"💾 Data saved to: {csv_filename}")
    except Exception as e:
        print(f"❌ Error saving CSV: {e}")
        return
    
    # Create plots
    try:
        # Extract data for plotting
        times = [row[1] for row in data]
        angles = [row[2] for row in data]
        
        # Unwrap angles to handle 0°/360° transitions
        unwrapped_angles = unwrap_angles(angles)
        
        # Calculate gradient (angular velocity) using unwrapped angles
        gradients = []
        gradient_times = []
        
        for i in range(1, len(unwrapped_angles)):
            dt = times[i] - times[i-1]
            dangle = unwrapped_angles[i] - unwrapped_angles[i-1]
            if dt > 0:  # Avoid division by zero
                gradient = dangle / dt  # degrees per second
                gradients.append(gradient)
                gradient_times.append(times[i])
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        fig.suptitle(f'Rotary Encoder Data - {duration:.2f}s Recording', fontsize=16)
        
        # Plot 1: Original angle over time (0-360°)
        ax1.plot(times, angles, 'b-', linewidth=1.5)
        ax1.set_ylabel('Angle (degrees)')
        ax1.set_title('Angle vs Time')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Gradient (angular velocity) over time (using unwrapped angles)
        if gradients:
            ax2.plot(gradient_times, gradients, 'r-', linewidth=1.5)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (deg/s)')
            ax2.set_title('Angular Velocity vs Time (Smooth)')
            ax2.grid(True, alpha=0.3)
            ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)  # Zero line
        
        plt.tight_layout()
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"📈 Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()
        
    except Exception as e:
        print(f"❌ Error creating plot: {e}")

def main():
    """Main function to connect to the encoder and read data."""
    # State variables for keyboard input
    z_key_was_pressed = False
    d_key_was_pressed = False
    j_key_was_pressed = False
    k_key_was_pressed = False
    is_ccw = False  # Start with default direction CW

    try:
        instrument = minimalmodbus.Instrument(PORT_NAME, SLAVE_ADDRESS)
        instrument.serial.baudrate = BAUDRATE
        instrument.serial.bytesize = 8
        instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
        instrument.serial.stopbits = 1
        instrument.serial.timeout = 0.5
        instrument.mode = minimalmodbus.MODE_RTU
        
        print(f"Successfully connected to encoder on {PORT_NAME}...")
        print("Controls:")
        print("  'z' - Zero encoder")
        print("  'd' - Toggle direction")
        print("  'j' - Start recording")
        print("  'k' - Stop recording and save data")
        print("  Ctrl+C - Stop program")
        
        # Set initial direction to CW
        set_direction(instrument, is_ccw)
        
        while True:
            # --- Handle Keyboard Input (Single press detection) ---

            # Zeroing ('z' key)
            if keyboard.is_pressed('z') and not z_key_was_pressed:
                zero_encoder(instrument)
                z_key_was_pressed = True
            elif not keyboard.is_pressed('z'):
                z_key_was_pressed = False

            # Direction Toggle ('d' key)
            if keyboard.is_pressed('d') and not d_key_was_pressed:
                is_ccw = set_direction(instrument, not is_ccw) # Toggle the state
                d_key_was_pressed = True
            elif not keyboard.is_pressed('d'):
                d_key_was_pressed = False

            # Start Recording ('j' key)
            if keyboard.is_pressed('j') and not j_key_was_pressed:
                start_recording()
                j_key_was_pressed = True
            elif not keyboard.is_pressed('j'):
                j_key_was_pressed = False

            # Stop Recording ('k' key)
            if keyboard.is_pressed('k') and not k_key_was_pressed:
                stop_recording()
                k_key_was_pressed = True
            elif not keyboard.is_pressed('k'):
                k_key_was_pressed = False

            # --- Read and Decode Encoder Data ---
            try:
                total_encoded_value = instrument.read_long(READ_REGISTER_ADDRESS, 3, False)
                single_turn_value = total_encoded_value & 0x7FFF
                turn_count = total_encoded_value >> 15
                angle = (single_turn_value / SINGLE_TURN_RESOLUTION) * 360.0
                
                direction_str = "CCW" if is_ccw else "CW"
                current_time = time.time()
                
                # Record data if recording is active
                if is_recording and recording_start_time is not None:
                    elapsed_time = current_time - recording_start_time
                    recorded_data.append([
                        datetime.now().strftime('%H:%M:%S.%f')[:-3],
                        elapsed_time,
                        angle,
                        turn_count,
                        single_turn_value,
                        total_encoded_value,
                        direction_str
                    ])
                
                # --- MODIFIED LINE ---
                # Added recording status and data count to the output string
                recording_status = "🔴" if is_recording else "⚪"
                data_count = len(recorded_data) if is_recording else 0
                print(
                    f"\r{recording_status} Dir: {direction_str} | Angle: {angle:7.2f}° | Turns: {turn_count:3d} | "
                    f"Single-Turn: {single_turn_value:5d} | Raw: {total_encoded_value:7d} | Data: {data_count:4d}   ",
                    end=""
                )
                
                time.sleep(0.05) 

            except minimalmodbus.ModbusException as e:
                # print(f"\rModbus error: {e}" + " "*40)
                time.sleep(1)
            except Exception as e:
                print(f"\nAn error occurred: {e}")
                break

    except Exception as e:
        print(f"Failed to connect on {PORT_NAME}. Error: {e}")

if __name__ == "__main__":
    main()