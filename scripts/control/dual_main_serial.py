import cv2
import numpy as np
import time
import sys
from dmrobotics import Sensor, put_arrows_on_image

# NEW IMPORT for finding hardware devices on Windows
try:
    import wmi
except ImportError:
    print("Error: The 'wmi' library is required. Please run: pip install wmi")
    sys.exit(1)

# Constants (from dual_main.py)
DISPLAY_SCALE = 20  # Scale factor for deformation and shear visualization
DEPTH_SCALE = 0.25  # Scale factor for depth visualization


# --- Configuration ---
# You have correctly labeled the unique serial IDs for your sensors.
# This dictionary is now the "source of truth" for the script.
SENSOR_SERIAL_IDS = {
    "sensor1": "7&1EBE4E51&0&0000",
    "sensor2": "7&21CAF26C&0&0000"
}

# --- NEW FUNCTION: Find all connected camera devices and their IDs ---
def find_camera_devices():
    """Uses WMI to find all connected UVC camera devices and their unique IDs."""
    print("Finding all connected camera devices...")
    c = wmi.WMI()
    devices = []
    # Query for camera devices using PNPClass
    for camera in c.Win32_PnPEntity(PNPClass="Camera"):
        if "USB" in camera.DeviceID:
            devices.append({"name": camera.Name, "id": camera.DeviceID})
    return devices

# --- NEW FUNCTION: Map sensor names to correct OpenCV indices ---
def map_sensors_by_id(target_ids, all_devices):
    """
    Matches the target serial IDs to the found devices and determines their
    correct OpenCV index.
    """
    print("Attempting to map sensors by their serial IDs...")
    
    # It is generally assumed that the device order from WMI and OpenCV are the same.
    # We will map the WMI device ID to the OpenCV index.
    device_id_to_index = {device['id']: i for i, device in enumerate(all_devices)}

    sensor_map = {}
    for name, partial_id in target_ids.items():
        found = False
        for full_id, index in device_id_to_index.items():
            if partial_id in full_id:
                sensor_map[name] = index
                print(f"  ✓ Found '{name}' at OpenCV index {index}")
                found = True
                break
        if not found:
            print(f"  ❌ Error: Could not find a connected sensor with ID containing '{partial_id}' for '{name}'")
            return None
            
    if len(sensor_map) != len(target_ids):
        print("Error: Could not find all specified sensors.")
        return None
        
    return sensor_map

def process_sensor_data(sensor):
    """Process and return data from a single sensor (from dual_main.py)"""
    img = sensor.getRawImage()
    deformation = sensor.getDeformation2D()
    shear = sensor.getShear()
    depth = sensor.getDepth()
    depth_img = cv2.applyColorMap((depth * DEPTH_SCALE * 255.0).astype('uint8'), cv2.COLORMAP_HOT)
    
    return {
        'img': img,
        'deformation': deformation,
        'shear': shear,
        'depth': depth,
        'depth_img': depth_img
    }

def display_sensor_data(sensor_num, data, black_img):
    """Process sensor data and return visualization images (simplified - no adjusted depth)"""
    # Process images
    deformation_img = put_arrows_on_image(black_img.copy(), data['deformation'] * DISPLAY_SCALE)
    shear_img = put_arrows_on_image(black_img.copy(), data['shear'] * DISPLAY_SCALE)
    
    # Add text labels to images
    cv2.putText(deformation_img, f"Sensor {sensor_num} - Deformation", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(shear_img, f"Sensor {sensor_num} - Shear", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(data['depth_img'], f"Sensor {sensor_num} - Raw Depth", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return {
        'deformation': deformation_img,
        'shear': shear_img,
        'raw_depth': data['depth_img']
    }

def hconcat_resize(img_list):
    """Concatenate images horizontally with consistent height (from dual_main.py)"""
    # Find the smallest height
    h_min = min(img.shape[0] for img in img_list)
    
    # Resize all images to have the same height
    resized_imgs = [cv2.resize(img, (int(img.shape[1] * h_min / img.shape[0]), h_min)) 
                   if img.shape[0] != h_min else img for img in img_list]
    
    # Concatenate the images horizontally
    return cv2.hconcat(resized_imgs)

def vconcat_resize(img_list):
    """Concatenate images vertically with consistent width (from dual_main.py)"""
    # Find the smallest width
    w_min = min(img.shape[1] for img in img_list)
    
    # Resize all images to have the same width
    resized_imgs = [cv2.resize(img, (w_min, int(img.shape[0] * w_min / img.shape[1]))) 
                   if img.shape[1] != w_min else img for img in img_list]
    
    # Concatenate the images vertically
    return cv2.vconcat(resized_imgs)

def main():
    # --- REVISED INITIALIZATION ---
    # 1. Find all physical camera devices connected to the system.
    all_devices = find_camera_devices()
    if not all_devices or len(all_devices) < 2:
        print("Error: Did not find at least two USB camera devices. Exiting.")
        return

    # 2. Create a reliable mapping from your names to the correct camera indices.
    sensor_map = map_sensors_by_id(SENSOR_SERIAL_IDS, all_devices)
    if sensor_map is None:
        print("Could not initialize sensors due to mapping failure. Exiting.")
        return

    # 3. Initialize your Sensor objects using the reliable indices.
    print("Initializing sensor objects...")
    try:
        sensor1 = Sensor(sensor_map['sensor1'])
        sensor2 = Sensor(sensor_map['sensor2'])
        print("✓ Sensors initialized successfully.")
    except Exception as e:
        print(f"Error during sensor initialization: {e}")
        return

    # --- Your existing logic adapted from dual_main.py ---
    # Create black backgrounds for visualization
    black_img1 = np.zeros_like(sensor1.getRawImage())
    black_img1 = np.stack([black_img1] * 3, axis=-1)
    
    black_img2 = np.zeros_like(sensor2.getRawImage())
    black_img2 = np.stack([black_img2] * 3, axis=-1)
    
    time.sleep(1)

    try:
        while True:
            # Process both sensors
            sensor1_data = process_sensor_data(sensor1)
            sensor2_data = process_sensor_data(sensor2)
            
            # Create visualizations
            sensor1_vis = display_sensor_data("1", sensor1_data, black_img1)
            sensor2_vis = display_sensor_data("2", sensor2_data, black_img2)

            # Create 3x2 arrangement:
            # sensor2 depth, sensor1 depth
            # sensor2 deformation, sensor1 deformation  
            # sensor2 shear, sensor1 shear
            depth_row = hconcat_resize([sensor2_vis['raw_depth'], sensor1_vis['raw_depth']])
            deformation_row = hconcat_resize([sensor2_vis['deformation'], sensor1_vis['deformation']])
            shear_row = hconcat_resize([sensor2_vis['shear'], sensor1_vis['shear']])
            
            # Combine all rows vertically
            combined_display = vconcat_resize([depth_row, deformation_row, shear_row])
            
            # Display the combined visualization
            cv2.imshow('Dual Sensor Display (Sensor2|Sensor1: Depth, Deformation, Shear)', combined_display)

            # Handle key events
            k = cv2.waitKey(3)
            if k & 0xFF == ord('q'):
                break
            elif k & 0xFF == ord('r'):
                sensor1.reset()
                sensor2.reset()
                print("Both sensors reset")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        sensor1.disconnect()
        sensor2.disconnect()
        cv2.destroyAllWindows()
        print("Sensors disconnected and windows closed.")

if __name__ == "__main__":
    main()