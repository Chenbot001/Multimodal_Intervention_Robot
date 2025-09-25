import os
import json
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from tqdm import tqdm

def create_full_timeseries_plot(elapsed_times, shear_x_values, shear_y_values, resultant_shear, filename):
    """
    Creates a full time series plot showing all shear force components.
    
    Args:
        elapsed_times (np.array): Array of elapsed times
        shear_x_values (np.array): Array of x-component shear values
        shear_y_values (np.array): Array of y-component shear values
        resultant_shear (np.array): Array of resultant shear magnitudes
        filename (str): Output filename for the plot image
    """
    print(f"📈 Creating full time series plot: '{filename}'...")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Top subplot: Individual components
    ax1.plot(elapsed_times, shear_x_values, 'r-', label='Shear X', alpha=0.7, linewidth=1.5)
    ax1.plot(elapsed_times, shear_y_values, 'b-', label='Shear Y', alpha=0.7, linewidth=1.5)
    ax1.set_title('Shear Force Components Over Time', fontsize=16)
    ax1.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax1.set_ylabel('Shear Force Components', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend()
    
    # Bottom subplot: Resultant magnitude
    ax2.plot(elapsed_times, resultant_shear, 'b-', label='Resultant Shear', linewidth=2)
    ax2.set_title('Resultant Shear Force Magnitude Over Time', fontsize=16)
    ax2.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax2.set_ylabel('Resultant Shear Force ($\\sqrt{S_x^2 + S_y^2}$)', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend()
    
    # Add statistics text
    stats_text = (
        f'Data Points: {len(elapsed_times)}\n'
        f'Duration: {elapsed_times[-1] - elapsed_times[0]:.2f}s\n'
        f'Resultant Range: {resultant_shear.min():.4f} to {resultant_shear.max():.4f}\n'
        f'Mean Resultant: {resultant_shear.mean():.4f}'
    )
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round,pad=0.5', fc='lightblue', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Full time series plot saved as '{filename}'")

def create_rolling_plot_video(json_folder, output_filename='rolling_resultant_shear_plot.mp4', window_seconds=10.0, fps=10, generate_full_plot=True):
    """
    Creates a video of a rolling plot from time-series JSON data showing resultant shear force.

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
    shear_x_values = []
    shear_y_values = []
    last_shear_x = 0.0  # Initialize with default values
    last_shear_y = 0.0
    print("📂 Reading and parsing JSON files...")
    for file_path in tqdm(json_files, desc="Parsing JSONs"):
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                elapsed_time = data['elapsed_time_s']
                shear_x = data['daimon']['shear_vector_x']
                shear_y = data['daimon']['shear_vector_y']
                
                # Handle None/null values by using last available values
                if elapsed_time is not None:
                    elapsed_times.append(elapsed_time)
                    
                    # Use last available value if current value is None
                    if shear_x is not None:
                        last_shear_x = shear_x
                        shear_x_values.append(shear_x)
                    else:
                        shear_x_values.append(last_shear_x)
                    
                    if shear_y is not None:
                        last_shear_y = shear_y
                        shear_y_values.append(shear_y)
                    else:
                        shear_y_values.append(last_shear_y)
        except (json.JSONDecodeError, KeyError, FileNotFoundError) as e:
            print(f"⚠️ Warning: Skipping file {os.path.basename(file_path)} due to error: {e}")
            continue

    if not elapsed_times:
        print("❌ Error: No valid data could be extracted.")
        return

    # --- 3. Prepare data for plotting ---
    elapsed_times = np.array(elapsed_times)
    shear_x_values = np.array(shear_x_values)
    shear_y_values = np.array(shear_y_values)
    
    # Calculate resultant shear force magnitude
    resultant_shear = np.sqrt(shear_x_values**2 + shear_y_values**2)
    
    # Sort data by elapsed time to ensure chronological order
    sort_indices = np.argsort(elapsed_times)
    elapsed_times = elapsed_times[sort_indices]
    shear_x_values = shear_x_values[sort_indices]
    shear_y_values = shear_y_values[sort_indices]
    resultant_shear = resultant_shear[sort_indices]
    
    print(f"📊 Loaded {len(elapsed_times)} data points spanning {elapsed_times[-1] - elapsed_times[0]:.2f} seconds")
    print(f"📈 Resultant shear range: {resultant_shear.min():.4f} to {resultant_shear.max():.4f}")
    # Note: elapsed_time_s is already relative to the start, so no need to normalize

    # --- 3.5. Generate full time series plot if requested ---
    if generate_full_plot:
        full_plot_filename = output_filename.replace('.mp4', '_full_timeseries.png')
        create_full_timeseries_plot(elapsed_times, shear_x_values, shear_y_values, resultant_shear, full_plot_filename)


    # --- 4. Set up the plot for animation ---
    fig, ax = plt.subplots(figsize=(12, 3))
    line_x, = ax.plot([], [], 'r-', lw=2, label='Shear X')
    line_y, = ax.plot([], [], 'b-', lw=2, label='Shear Y')

    # Style the plot
    ax.set_title(f'Rolling Plot of Shear Force Components - {window_seconds}s Window', fontsize=16)
    ax.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax.set_ylabel('Shear Force', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()
    fig.tight_layout()

    # Set fixed y-limits for a stable view, with a 10% margin
    y_min = min(shear_x_values.min(), shear_y_values.min())
    y_max = max(shear_x_values.max(), shear_y_values.max())
    y_margin = (y_max - y_min) * 0.1
    ax.set_ylim(y_min - y_margin, y_max + y_margin)

    # Add a text element to display the current time
    time_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, fontsize=12, 
                        bbox=dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.5))

    # --- 5. Define the animation function ---
    def update(frame):
        # Calculate the start and end time for the rolling window
        current_time = elapsed_times[frame]
        window_start_time = max(0, current_time - window_seconds)

        # Find the indices of the data that fall within this window
        indices = np.where((elapsed_times >= window_start_time) & (elapsed_times <= current_time))

        # Get the data for the plot
        x_data = elapsed_times[indices]
        y_data_x = shear_x_values[indices]
        y_data_y = shear_y_values[indices]

        # Update the plot lines and axis limits
        line_x.set_data(x_data, y_data_x)
        line_y.set_data(x_data, y_data_y)
        ax.set_xlim(float(window_start_time), float(current_time) + 0.01)
        time_text.set_text(f'Elapsed Time: {current_time:.2f} s')

        return line_x, line_y, time_text

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


if __name__ == '__main__':
    # --- HOW TO USE ---
    # 1. Place this script in a folder.
    # 2. Create a subfolder named 'my_json_data' (or any name you like).
    # 3. Put all your JSON files into the 'my_json_data' subfolder.
    # 4. Update the 'json_data_folder' variable below to match your folder's name.
    # 5. Run the script!

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
            data = {
                "timestamp": ts,
                "elapsed_time_s": elapsed_time,
                "daimon": { 
                    "shear_vector_x": np.cos(i / 10) * 0.3 + (np.random.rand() - 0.5) * 0.1,
                    "shear_vector_y": np.sin(i / 10) * 0.5 + (np.random.rand() - 0.5) * 0.1 
                }
            }
            with open(os.path.join(json_data_folder, file_name), 'w') as f:
                json.dump(data, f)
        print("Generated 200 dummy JSON files.")

    # Call the main function with realistic fps matching your data collection rate
    # If your data was collected at ~10Hz, use fps=10
    # If collected at ~30Hz, use fps=30 for real-time playback
    create_rolling_plot_video(json_folder=json_data_folder, 
                            output_filename="resultant_shear_rolling_plot.mp4",
                            fps=10)  # Adjust this to match your data collection frequency