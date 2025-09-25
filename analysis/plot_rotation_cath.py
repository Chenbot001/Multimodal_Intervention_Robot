import os
import json
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from tqdm import tqdm

def calculate_induced_rotation(m1_stepper_position, centerline_angle_offset, gripper_closure_percent, 
                              gripper_state_tracker, elapsed_time=None):
    """
    Calculate the induced rotation in degrees using the dual regime control equation.
    Considers gripper closure state and time-based object/regime switching for rotation calculation.
    
    For OC (open configuration) regime:
    Θ_deg(d, n, θ) = 0.53 * (720sn/πd) * cos(θ) - 0.43
    
    For CC (closed configuration) regime:
    Θ_deg(d, n, θ) = 0.53 * (720sn/πd) - 11.73/d - 0.55
    
    Args:
        m1_stepper_position (float): Current stepper motor position
        centerline_angle_offset (float): Centerline angle offset in degrees
        gripper_closure_percent (float): Gripper closure percentage
        gripper_state_tracker (dict): Tracker for gripper state and initial positions
        elapsed_time (float): Elapsed time in seconds for object/regime determination
    
    Returns:
        tuple: (induced_rotation_degrees, updated_gripper_state_tracker)
    """
    # Constants
    s = 0.0254  # Step travel in mm/step
    
    # Determine object diameter and regime based on elapsed time
    if elapsed_time is not None and elapsed_time >= 150.0:
        # From 150s onwards: guidewire (0.3mm) in CC regime
        d = 0.3
        regime = 'CC'
    else:
        # First 150s: catheter (2.0mm) in OC regime
        d = 2.0
        regime = 'OC'
    # Determine object diameter and regime based on elapsed time
    if elapsed_time is not None and elapsed_time >= 150.0:
        # From 150s onwards: guidewire (0.3mm) in CC regime
        d = 0.3
        regime = 'CC'
    else:
        # First 150s: catheter (2.0mm) in OC regime
        d = 2.0
        regime = 'OC'
    
    # Update gripper state tracker
    previous_state = gripper_state_tracker.get('is_closed', False)
    current_is_closed = gripper_closure_percent >= 95  # Gripper is closed at 95%
    current_is_open = gripper_closure_percent <= 50    # Gripper is open at 50%
    
    # Track initial position when gripper transitions from open to closed
    if current_is_open and not gripper_state_tracker.get('is_closed', False):
        # Gripper is open, continuously update the tracked initial position
        gripper_state_tracker['tracked_initial_position'] = m1_stepper_position
        gripper_state_tracker['is_closed'] = False
        
    elif current_is_closed and not previous_state:
        # Gripper just closed (transition from open to closed)
        # Use the last tracked position as the new initial position
        gripper_state_tracker['current_initial_position'] = gripper_state_tracker.get('tracked_initial_position', 500)
        gripper_state_tracker['is_closed'] = True
        
    elif current_is_closed:
        # Gripper remains closed, keep current state
        gripper_state_tracker['is_closed'] = True
        
    # Both OC and CC regimes only apply when gripper is closed (≥95%)
    # When gripper is open, no rotation is induced regardless of regime
    if not gripper_state_tracker.get('is_closed', False):
        return 0.0, gripper_state_tracker
    
    # Calculate n (number of steps from the dynamic initial position)
    initial_position = gripper_state_tracker.get('current_initial_position', 500)
    n = m1_stepper_position - initial_position
    
    # No rotation if no stepper movement has occurred
    if n == 0:
        return 0.0, gripper_state_tracker
    
    # Apply the appropriate regime equation
    if regime == 'OC':
        # OC regime equation: Θ_deg(d, n, θ) = 0.53 * (720sn/πd) * cos(θ) - 0.43
        # Convert centerline_angle_offset from degrees to radians for cos calculation
        theta_rad = np.deg2rad(centerline_angle_offset if centerline_angle_offset is not None else 0.0)
        theta_deg = 0.53 * (720 * s * n / (np.pi * d)) * np.cos(theta_rad) - 0.43
    else:  # CC regime
        # CC regime equation: Θ_deg(d, n, θ) = 0.53 * (720sn/πd) - 11.73/d - 0.55
        theta_deg = 0.53 * (720 * s * n / (np.pi * d)) - 11.73 / d - 0.55
    
    return -theta_deg, gripper_state_tracker

def create_full_timeseries_plot(elapsed_times, rotation_values, gripper_closures, filename):
    """
    Creates a full time series plot showing induced rotation over time with gripper state and regime information.
    
    Args:
        elapsed_times (np.array): Array of elapsed times
        rotation_values (np.array): Array of induced rotation values in degrees
        gripper_closures (np.array): Array of gripper closure percentages
        filename (str): Output filename for the plot image
    """
    print(f"📈 Creating full time series plot: '{filename}'...")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 5))
    
    # Top subplot: Induced rotation
    ax1.plot(elapsed_times, rotation_values, 'b-', linewidth=2, alpha=0.8)
    ax1.set_title('Induced Rotation Over Time (Dual Regime)', fontsize=16)
    ax1.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax1.set_ylabel('Induced Rotation (degrees)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.6)
    
    # Add regime transition line
    ax1.axvline(x=150, color='red', linestyle='--', alpha=0.8, linewidth=2, label='Regime Transition (150s)')
    
    # Highlight periods based on regime
    oc_mask = elapsed_times < 150
    cc_mask = elapsed_times >= 150
    
    if np.any(oc_mask):
        ax1.fill_between(elapsed_times, ax1.get_ylim()[0], ax1.get_ylim()[1], 
                        where=oc_mask, alpha=0.15, color='orange', 
                        label='OC Regime (Catheter 2.0mm)')
    if np.any(cc_mask):
        ax1.fill_between(elapsed_times, ax1.get_ylim()[0], ax1.get_ylim()[1], 
                        where=cc_mask, alpha=0.15, color='blue', 
                        label='CC Regime (Guidewire 0.3mm)')
    
    # Highlight gripper closed periods (when rotation can occur in either regime)
    closed_mask = gripper_closures >= 95
    if np.any(closed_mask):
        ax1.fill_between(elapsed_times, ax1.get_ylim()[0], ax1.get_ylim()[1], 
                        where=closed_mask, alpha=0.3, color='green', 
                        label='Gripper Closed (Rotation Active)')
    
    ax1.legend()
    
    # Bottom subplot: Gripper closure percentage
    ax2.plot(elapsed_times, gripper_closures, 'r-', linewidth=2, alpha=0.8)
    ax2.axhline(y=50, color='orange', linestyle='--', alpha=0.7, label='Open Threshold (50%)')
    ax2.axhline(y=95, color='green', linestyle='--', alpha=0.7, label='Closed Threshold (95%)')
    ax2.axvline(x=150, color='red', linestyle='--', alpha=0.8, linewidth=2, label='Regime Transition (150s)')
    ax2.set_title('Gripper Closure Percentage Over Time', fontsize=16)
    ax2.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax2.set_ylabel('Gripper Closure (%)', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend()
    ax2.set_ylim(0, 100)
    
    # Add statistics text
    non_zero_rotation = rotation_values[rotation_values != 0]
    oc_rotations = rotation_values[oc_mask]
    cc_rotations = rotation_values[cc_mask]
    
    stats_text = (
        f'Data Points: {len(elapsed_times)}\n'
        f'Duration: {elapsed_times[-1] - elapsed_times[0]:.2f}s\n'
        f'Rotation Range: {rotation_values.min():.2f}° to {rotation_values.max():.2f}°\n'
        f'Active Rotation Points: {len(non_zero_rotation)}\n'
        f'OC Period (0-150s): {len(oc_rotations)} points\n'
        f'CC Period (150s+): {len(cc_rotations)} points\n'
        f'Total Rotation Change: {rotation_values[-1] - rotation_values[0]:.2f}°'
    )
    ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round,pad=0.5', fc='lightgreen', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Full time series plot saved as '{filename}'")

def create_rolling_plot_video(json_folder, output_filename='rolling_rotation_plot.mp4', window_seconds=10.0, fps=10, generate_full_plot=True):
    """
    Creates a video of a rolling plot from time-series JSON data showing induced rotation.

    Args:
        json_folder (str): Path to the folder containing the JSON files.
        output_filename (str): Name of the output .mp4 video file.
        window_seconds (float): The duration of the rolling window in seconds.
        fps (int): Frames per second for the output video, should match data frequency.
        generate_full_plot (bool): Whether to also generate a full time series plot image.
    """
    # --- 1. Find and sort JSON files ---
    print(f"🔍 Searching for JSON files in '{json_folder}'...")
    json_files = sorted(glob.glob(os.path.join(json_folder, '*.json')))
    if not json_files:
        print(f"❌ Error: No JSON files found in the specified folder.")
        return

    # --- 2. Extract data from files ---
    elapsed_times = []
    rotation_values = []
    gripper_closures = []
    last_m1_position = 500  # Initialize with default starting position
    last_centerline_offset = 0.0  # Initialize with default value
    last_gripper_closure = 0.0  # Initialize with default value
    
    # Initialize gripper state tracker
    gripper_state_tracker = {
        'is_closed': False,
        'tracked_initial_position': 500,
        'current_initial_position': 500
    }
    
    print("📂 Reading and parsing JSON files...")
    for file_path in tqdm(json_files, desc="Parsing JSONs"):
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                elapsed_time = data['elapsed_time_s']
                
                # Extract gripper data
                gripper_data = data.get('gripper', {})
                m1_stepper_position = gripper_data.get('m1_stepper_position')
                gripper_closure_percent = gripper_data.get('gripper_closure_percent')
                
                # Extract daimon data
                daimon_data = data.get('daimon', {})
                centerline_angle_offset = daimon_data.get('centerline_angle_offset')
                
                # Handle None/null values by using last available values
                if elapsed_time is not None:
                    elapsed_times.append(elapsed_time)
                    
                    # Use last available value if current value is None
                    if m1_stepper_position is not None:
                        last_m1_position = m1_stepper_position
                    if centerline_angle_offset is not None:
                        last_centerline_offset = centerline_angle_offset
                    if gripper_closure_percent is not None:
                        last_gripper_closure = gripper_closure_percent
                    
                    # Calculate induced rotation using current or last available values
                    induced_rotation, gripper_state_tracker = calculate_induced_rotation(
                        last_m1_position, 
                        last_centerline_offset, 
                        last_gripper_closure,
                        gripper_state_tracker,
                        elapsed_time
                    )
                    rotation_values.append(induced_rotation)
                    gripper_closures.append(last_gripper_closure)
                    
        except (json.JSONDecodeError, KeyError, FileNotFoundError) as e:
            print(f"⚠️ Warning: Skipping file {os.path.basename(file_path)} due to error: {e}")
            continue

    if not elapsed_times:
        print("❌ Error: No valid data could be extracted.")
        return

    # --- 3. Prepare data for plotting ---
    elapsed_times = np.array(elapsed_times)
    rotation_values = np.array(rotation_values)
    gripper_closures = np.array(gripper_closures)
    
    # Sort data by elapsed time to ensure chronological order
    sort_indices = np.argsort(elapsed_times)
    elapsed_times = elapsed_times[sort_indices]
    rotation_values = rotation_values[sort_indices]
    gripper_closures = gripper_closures[sort_indices]
    
    print(f"📊 Loaded {len(elapsed_times)} data points spanning {elapsed_times[-1] - elapsed_times[0]:.2f} seconds")
    print(f"📈 Rotation range: {rotation_values.min():.2f}° to {rotation_values.max():.2f}°")

    # Calculate regime-specific y-axis ranges for dynamic scaling
    oc_mask = elapsed_times < 150.0
    cc_mask = elapsed_times >= 150.0
    
    # Calculate OC regime range
    if np.any(oc_mask):
        oc_rotations = rotation_values[oc_mask]
        oc_non_zero = oc_rotations[oc_rotations != 0]
        if len(oc_non_zero) > 0:
            oc_min, oc_max = oc_non_zero.min(), oc_non_zero.max()
            oc_margin = max((oc_max - oc_min) * 0.2, 1.0)  # 20% margin, minimum 1 degree
            oc_y_range = (oc_min - oc_margin, oc_max + oc_margin)
        else:
            oc_y_range = (-1.0, 1.0)  # Default range if no OC data
    else:
        oc_y_range = (-1.0, 1.0)
    
    # Calculate CC regime range
    if np.any(cc_mask):
        cc_rotations = rotation_values[cc_mask]
        cc_non_zero = cc_rotations[cc_rotations != 0]
        if len(cc_non_zero) > 0:
            cc_min, cc_max = cc_non_zero.min(), cc_non_zero.max()
            cc_margin = max((cc_max - cc_min) * 0.2, 1.0)  # 20% margin, minimum 1 degree
            cc_y_range = (cc_min - cc_margin, cc_max + cc_margin)
        else:
            cc_y_range = (-1.0, 1.0)  # Default range if no CC data
    else:
        cc_y_range = (-1.0, 1.0)
    
    print(f"📊 Dynamic Y-axis ranges:")
    print(f"   • OC regime (0-150s): {oc_y_range[0]:.2f}° to {oc_y_range[1]:.2f}°")
    print(f"   • CC regime (150s+): {cc_y_range[0]:.2f}° to {cc_y_range[1]:.2f}°")

    # --- 3.5. Generate full time series plot if requested ---
    if generate_full_plot:
        full_plot_filename = output_filename.replace('.mp4', '_full_timeseries.png')
        create_full_timeseries_plot(elapsed_times, rotation_values, gripper_closures, full_plot_filename)

    # --- 4. Set up the plot for animation ---
    fig, ax = plt.subplots(figsize=(12, 3))
    line, = ax.plot([], [], 'b-', lw=2, label='Induced Rotation')

    # Style the plot
    ax.set_title(f'Rolling Plot of Induced Rotation - {window_seconds}s Window', fontsize=16)
    ax.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax.set_ylabel('Induced Rotation (degrees)', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()
    fig.tight_layout()

    # Initialize with OC regime y-limits (will be updated dynamically)
    current_y_range = oc_y_range
    ax.set_ylim(current_y_range[0], current_y_range[1])

    # Add text elements to display current information
    time_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, fontsize=10, 
                        bbox=dict(boxstyle='round,pad=0.2', fc='wheat', alpha=0.5))
    rotation_text = ax.text(0.02, 0.75, '', transform=ax.transAxes, fontsize=10,
                           bbox=dict(boxstyle='round,pad=0.2', fc='lightblue', alpha=0.5))
    gripper_text = ax.text(0.02, 0.60, '', transform=ax.transAxes, fontsize=10,
                          bbox=dict(boxstyle='round,pad=0.2', fc='lightgreen', alpha=0.5))
    regime_text = ax.text(0.02, 0.45, '', transform=ax.transAxes, fontsize=10,
                         bbox=dict(boxstyle='round,pad=0.2', fc='lightyellow', alpha=0.5))

    # --- 5. Define the animation function ---
    def update(frame):
        nonlocal current_y_range
        
        # Calculate the start and end time for the rolling window
        current_time = elapsed_times[frame]
        window_start_time = max(0, current_time - window_seconds)

        # Find the indices of the data that fall within this window
        indices = np.where((elapsed_times >= window_start_time) & (elapsed_times <= current_time))

        # Get the data for the plot
        x_data = elapsed_times[indices]
        y_data = rotation_values[indices]

        # Update the plot line and axis limits
        line.set_data(x_data, y_data)
        ax.set_xlim(float(window_start_time), float(current_time) + 0.01)
        
        # Determine current regime and update y-axis if needed
        if current_time >= 150.0:
            target_y_range = cc_y_range
            regime = 'CC'
            object_info = 'Guidewire (0.3mm)'
            regime_color = 'lightblue'
        else:
            target_y_range = oc_y_range
            regime = 'OC'
            object_info = 'Catheter (2.0mm)'
            regime_color = 'lightyellow'
        
        # Smoothly transition y-axis limits if regime changed
        if target_y_range != current_y_range:
            # Interpolation factor for smooth transition (adjust for faster/slower transition)
            alpha = 0.1  # 0.1 = slow transition, 0.5 = medium, 1.0 = instant
            new_y_min = current_y_range[0] + alpha * (target_y_range[0] - current_y_range[0])
            new_y_max = current_y_range[1] + alpha * (target_y_range[1] - current_y_range[1])
            current_y_range = (new_y_min, new_y_max)
            ax.set_ylim(current_y_range[0], current_y_range[1])
        
        # Update text displays
        current_rotation = rotation_values[frame]
        current_gripper_closure = gripper_closures[frame]
        
        gripper_state = "CLOSED" if current_gripper_closure >= 95 else "OPEN" if current_gripper_closure <= 50 else "TRANSITIONING"
        gripper_color = 'lightgreen' if current_gripper_closure >= 95 else 'orange' if current_gripper_closure <= 50 else 'yellow'
        
        time_text.set_text(f'Time: {current_time:.2f} s')
        rotation_text.set_text(f'Rotation: {current_rotation:.2f}°')
        gripper_text.set_text(f'Gripper: {gripper_state}')
        gripper_text.set_bbox(dict(boxstyle='round,pad=0.3', fc=gripper_color, alpha=0.5))
        regime_text.set_text(f'Regime: {regime} | {object_info}')
        regime_text.set_bbox(dict(boxstyle='round,pad=0.3', fc=regime_color, alpha=0.5))

        return line, time_text, rotation_text, gripper_text, regime_text

    # --- 6. Create and save the animation ---
    num_frames = len(elapsed_times)
    ani = FuncAnimation(fig, update, frames=num_frames, blit=True, interval=1000/fps)

    print(f"\n🎥 Rendering video to '{output_filename}'...")
    writer = FFMpegWriter(fps=fps, metadata=dict(artist='Gemini'), bitrate=1800)

    # Wrap the save process with a tqdm progress bar
    progress_bar = tqdm(total=num_frames, desc="Rendering Frames", unit='frame')
    ani.save(output_filename, writer=writer, progress_callback=lambda i, n: progress_bar.update(1))
    progress_bar.close()

    plt.close(fig) # Prevent the final plot from displaying
    print(f"\n✅ Video saved successfully as '{output_filename}'.")

def analyze_rotation_data(json_folder):
    """
    Analyze the rotation data and provide summary statistics.
    
    Args:
        json_folder (str): Path to the folder containing the JSON files.
    """
    print(f"\n📊 Analyzing rotation data from '{json_folder}'...")
    
    json_files = sorted(glob.glob(os.path.join(json_folder, '*.json')))
    if not json_files:
        print("❌ No JSON files found.")
        return
    
    elapsed_times = []
    rotation_values = []
    stepper_positions = []
    centerline_offsets = []
    gripper_closures = []
    
    # Initialize gripper state tracker for analysis
    gripper_state_tracker = {
        'is_closed': False,
        'tracked_initial_position': 500,
        'current_initial_position': 500
    }
    
    for file_path in json_files:
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                elapsed_time = data.get('elapsed_time_s')
                
                if elapsed_time is not None:
                    gripper_data = data.get('gripper', {})
                    daimon_data = data.get('daimon', {})
                    
                    m1_position = gripper_data.get('m1_stepper_position', 500)
                    centerline_offset = daimon_data.get('centerline_angle_offset', 0.0)
                    gripper_closure = gripper_data.get('gripper_closure_percent', 0.0)
                    
                    rotation, gripper_state_tracker = calculate_induced_rotation(
                        m1_position, centerline_offset, gripper_closure, gripper_state_tracker, elapsed_time
                    )
                    
                    elapsed_times.append(elapsed_time)
                    rotation_values.append(rotation)
                    stepper_positions.append(m1_position)
                    centerline_offsets.append(centerline_offset if centerline_offset is not None else 0.0)
                    gripper_closures.append(gripper_closure)
                    
        except (json.JSONDecodeError, KeyError) as e:
            continue
    
    if rotation_values:
        rotation_values = np.array(rotation_values)
        stepper_positions = np.array(stepper_positions)
        gripper_closures = np.array(gripper_closures)
        elapsed_times = np.array(elapsed_times)
        
        # Analyze gripper state changes
        closed_indices = gripper_closures >= 95
        open_indices = gripper_closures <= 50
        
        # Analyze regime periods
        oc_period_mask = elapsed_times < 150.0
        cc_period_mask = elapsed_times >= 150.0
        
        print(f"📈 Rotation Analysis:")
        print(f"   • Total data points: {len(rotation_values)}")
        print(f"   • Rotation range: {rotation_values.min():.2f}° to {rotation_values.max():.2f}°")
        print(f"   • Mean rotation: {rotation_values.mean():.2f}°")
        print(f"   • Rotation std dev: {rotation_values.std():.2f}°")
        print(f"   • Total rotation change: {rotation_values[-1] - rotation_values[0]:.2f}°")
        print(f"   • Stepper position range: {stepper_positions.min()} to {stepper_positions.max()}")
        print(f"   • Steps from initial (500): {stepper_positions.min() - 500} to {stepper_positions.max() - 500}")
        print(f"🤏 Gripper State Analysis:")
        print(f"   • Gripper closure range: {gripper_closures.min():.1f}% to {gripper_closures.max():.1f}%")
        print(f"   • Time with gripper closed (≥95%): {np.sum(closed_indices)} data points")
        print(f"   • Time with gripper open (≤50%): {np.sum(open_indices)} data points")
        print(f"   • Non-zero rotation data points: {np.sum(rotation_values != 0)}")
        print(f"⚙️ Regime Analysis (Both require gripper closed for rotation):")
        print(f"   • OC period (0-150s): {np.sum(oc_period_mask)} data points")
        print(f"   • CC period (150s+): {np.sum(cc_period_mask)} data points")
        if np.any(oc_period_mask):
            oc_rotations = rotation_values[oc_period_mask]
            oc_closed_rotations = rotation_values[oc_period_mask & closed_indices]
            print(f"   • OC regime rotation range: {oc_rotations.min():.2f}° to {oc_rotations.max():.2f}°")
            print(f"   • OC regime non-zero rotations: {np.sum(oc_rotations != 0)} (when gripper closed: {np.sum(oc_closed_rotations != 0)})")
        if np.any(cc_period_mask):
            cc_rotations = rotation_values[cc_period_mask]
            cc_closed_rotations = rotation_values[cc_period_mask & closed_indices]
            print(f"   • CC regime rotation range: {cc_rotations.min():.2f}° to {cc_rotations.max():.2f}°")
            print(f"   • CC regime non-zero rotations: {np.sum(cc_rotations != 0)} (when gripper closed: {np.sum(cc_closed_rotations != 0)})")
        print(f"   • Final gripper state tracker: {gripper_state_tracker}")

if __name__ == '__main__':
    # --- HOW TO USE ---
    # 1. Place this script in a folder.
    # 2. Create a subfolder with your JSON data files.
    # 3. Update the 'json_data_folder' variable below to match your folder's name.
    # 4. Run the script!

    json_data_folder = 'demo/catheter_1757712728' # <-- IMPORTANT: Change this to your folder name

    # Create a dummy folder with sample data if it doesn't exist
    if not os.path.exists(json_data_folder):
        print(f"'{json_data_folder}' not found. Creating dummy data for demonstration.")
        os.makedirs(json_data_folder)
        start_ts = 1757717044.0
        for i in range(200): # Create 20 seconds of data
            ts = start_ts + i * 0.1
            elapsed_time = i * 0.1  # Elapsed time in seconds
            file_name = f"data_point_{i:04d}.json"
            
            # Simulate stepper motor movement (gradual change from 500)
            stepper_pos = 500 + int(i * 0.5)  # Gradual increase
            centerline_offset = np.sin(i / 20) * 5.0  # Oscillating centerline offset
            gripper_closure = min(100, i * 2)  # Gradual closure
            
            data = {
                "timestamp": ts,
                "elapsed_time_s": elapsed_time,
                "gripper": {
                    "m1_stepper_position": stepper_pos,
                    "gripper_closure_percent": gripper_closure,
                    "m2_stepper_position": 500,
                    "step_size": 20,
                    "speed_steps_per_sec": 50
                },
                "daimon": {
                    "centerline_angle_offset": centerline_offset,
                    "shear_vector_x": np.random.rand() * 0.1,
                    "shear_vector_y": np.random.rand() * 0.1
                }
            }
            with open(os.path.join(json_data_folder, file_name), 'w') as f:
                json.dump(data, f)
        print("Generated 200 dummy JSON files with rotation data.")

    # Analyze the rotation data first
    analyze_rotation_data(json_data_folder)
    
    # Call the main function with realistic fps matching your data collection rate
    # If your data was collected at ~10Hz, use fps=10
    # If collected at ~30Hz, use fps=30 for real-time playback
    create_rolling_plot_video(json_folder=json_data_folder, 
                            output_filename="induced_rotation_cath.mp4",
                            fps=10)  # Adjust this to match your data collection frequency