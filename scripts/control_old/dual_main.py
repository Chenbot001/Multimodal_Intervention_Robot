#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Dual Tactile Sensor Interface
------------------------------
This script accesses and displays results from two DM-Tac tactile sensors simultaneously.
It provides visualization of depth, raw image, deformation, and shear data.

Serial IDs:
- Sensor 1: 1ebe4e51 (Device ID 0)
- Sensor 2: 21caf26c (Device ID 1)

Keys:
- 'q': Exit the program
- 'r': Reset the sensors
- Ctrl+C: Graceful shutdown
"""

import time
import cv2
import numpy as np
import signal
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'sensors', 'daimon'))
from dmrobotics import Sensor, put_arrows_on_image

# Constants
DISPLAY_SCALE = 20  # Scale factor for deformation and shear visualization
DEPTH_SCALE = 0.25  # Scale factor for depth visualization 
FPS_UPDATE_INTERVAL = 1.0  # Seconds between FPS updates

def signal_handler(sig, frame):
    """Handle keyboard interrupt (Ctrl+C) gracefully"""
    print('\nKeyboard interrupt received. Shutting down gracefully...')
    sys.exit(0)

def hconcat_resize(img_list):
    """Concatenate images horizontally with consistent height"""
    # Find the smallest height
    h_min = min(img.shape[0] for img in img_list)
    
    # Resize all images to have the same height
    resized_imgs = [cv2.resize(img, (int(img.shape[1] * h_min / img.shape[0]), h_min)) 
                   if img.shape[0] != h_min else img for img in img_list]
    
    # Concatenate the images horizontally
    return cv2.hconcat(resized_imgs)

def vconcat_resize(img_list):
    """Concatenate images vertically with consistent width"""
    # Find the smallest width
    w_min = min(img.shape[1] for img in img_list)
    
    # Resize all images to have the same width
    resized_imgs = [cv2.resize(img, (w_min, int(img.shape[0] * w_min / img.shape[1]))) 
                   if img.shape[1] != w_min else img for img in img_list]
    
    # Concatenate the images vertically
    return cv2.vconcat(resized_imgs)

def create_grid(img_top_row, img_bottom_row):
    """Create a 2x2 grid of images"""
    # Concatenate images horizontally for each row
    top_row = hconcat_resize(img_top_row)
    bottom_row = hconcat_resize(img_bottom_row)
    
    # Concatenate the rows vertically
    return vconcat_resize([top_row, bottom_row])

def process_sensor_data(sensor, black_img, frame_num):
    """Process and return data from a single sensor"""
    img = sensor.getRawImage()
    deformation = sensor.getDeformation2D()
    shear = sensor.getShear()
    depth = sensor.getDepth()
    depth_img = cv2.applyColorMap((depth * DEPTH_SCALE * 255.0).astype('uint8'), cv2.COLORMAP_HOT)
    
    return {
        'img': img,
        'deformation': deformation,
        'shear': shear,
        'depth_img': depth_img
    }

def display_sensor_data(sensor_num, data, black_img):
    """Process sensor data and return visualization images"""
    # Process images
    deformation_img = put_arrows_on_image(black_img.copy(), data['deformation'] * DISPLAY_SCALE)
    shear_img = put_arrows_on_image(black_img.copy(), data['shear'] * DISPLAY_SCALE)
    
    # Add text labels to images
    cv2.putText(deformation_img, f"Sensor {sensor_num} - Deformation", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(shear_img, f"Sensor {sensor_num} - Shear", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Add text label to depth image
    depth_img_labeled = data['depth_img'].copy()
    cv2.putText(depth_img_labeled, f"Sensor {sensor_num} - Depth", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return {
        'deformation': deformation_img,
        'shear': shear_img,
        'depth': depth_img_labeled,
        # Uncomment this line if you want to display raw images
        # 'raw': data['img']
    }

def display_combined_data(sensor1_vis, sensor2_vis=None):
    """Display sensor visualizations in a 2x3 grid"""
    # Create a placeholder black image for single sensor mode
    if not sensor2_vis:
        # Make a placeholder with same dimensions as sensor1's images
        placeholder = np.zeros_like(sensor1_vis['deformation'])
        cv2.putText(placeholder, "No Sensor 2 Data", (10, placeholder.shape[0]//2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        sensor2_vis = {
            'deformation': placeholder,
            'shear': placeholder.copy(),
            'depth': placeholder.copy()
        }
    
    # Create the 2x3 grid (deformation, shear, depth images)
    top_row = [sensor1_vis['deformation'], sensor1_vis['shear'], sensor1_vis['depth']]
    bottom_row = [sensor2_vis['deformation'], sensor2_vis['shear'], sensor2_vis['depth']]
    
    # Combine all images into a single grid
    grid_img = create_grid(top_row, bottom_row)
    
    # Display the combined visualization
    cv2.imshow('Tactile Sensor Visualization', grid_img)
    
    # Uncomment these lines if you want to display raw images
    # if 'raw' in sensor1_vis:
    #     raw_images = [sensor1_vis['raw']]
    #     if sensor2_vis and 'raw' in sensor2_vis:
    #         raw_images.append(sensor2_vis['raw'])
    #     combined_raw = hconcat_resize(raw_images)
    #     cv2.imshow('Raw Image Visualization', combined_raw)

if __name__ == "__main__":
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Release any existing cameras that might be open
    cv2.destroyAllWindows()
    
    # Initialize sensors with error handling
    try:
        sensor1 = Sensor(0)  # First sensor (21CAF26C)
        print("Sensor 1 (ID: 0) initialized successfully")
        time.sleep(2.5)  # Allow time for first sensor to fully initialize
    except Exception as e:
        print(f"Failed to initialize sensor 1: {e}")
        exit(1)
    
    try:
        sensor2 = Sensor(1)  # Second sensor (1EBE4E51)
        print("Sensor 2 (ID: 1) initialized successfully")
        dual_sensor_mode = True
    except Exception as e:
        print(f"Failed to initialize sensor 2: {e}")
        print("Running in single sensor mode with sensor 1 only")
        sensor2 = None
        dual_sensor_mode = False

    # Initialize timing and display variables
    frame_num1, frame_num2 = 0.0, 0.0
    start_time1, start_time2 = time.time(), time.time()

    # Create black backgrounds for visualization
    black_img1 = np.zeros_like(sensor1.getRawImage())
    black_img1 = np.stack([black_img1] * 3, axis=-1)

    if dual_sensor_mode:
        black_img2 = np.zeros_like(sensor2.getRawImage())
        black_img2 = np.stack([black_img2] * 3, axis=-1)    
        
    try:
        while True:
            # Process Sensor 1 data
            frame_num1 += 1.0
            sensor1_data = process_sensor_data(sensor1, black_img1, frame_num1)
            sensor1_vis = display_sensor_data(1, sensor1_data, black_img1)

            # Process Sensor 2 data (if available)
            sensor2_vis = None
            if dual_sensor_mode:
                frame_num2 += 1.0
                sensor2_data = process_sensor_data(sensor2, black_img2, frame_num2)
                sensor2_vis = display_sensor_data(2, sensor2_data, black_img2)
            
            # Display combined visualizations
            display_combined_data(sensor1_vis, sensor2_vis)

            # Handle key events
            k = cv2.waitKey(3)
            if k & 0xFF == ord('q'):
                break
            elif k & 0xFF == ord('r'):
                sensor1.reset()
                if dual_sensor_mode:
                    sensor2.reset()
                    print("Both sensors reset")
                else:
                    print("Sensor 1 reset")

            # FPS calculation and display for Sensor 1
            current_time = time.time()
            if current_time - start_time1 > FPS_UPDATE_INTERVAL:
                fps1 = frame_num1 / (current_time - start_time1)
                if dual_sensor_mode:
                    print(f"Sensor 1 FPS: {fps1:.2f}", end=' | ')
                else:
                    print(f"Sensor 1 FPS: {fps1:.2f}", end='\r')
                frame_num1 = 0.0
                start_time1 = current_time
                
            # FPS calculation and display for Sensor 2 (if available)
            if dual_sensor_mode and time.time() - start_time2 > FPS_UPDATE_INTERVAL:
                fps2 = frame_num2 / (time.time() - start_time2)
                print(f"Sensor 2 FPS: {fps2:.2f}", end='\r')
                frame_num2 = 0.0
                start_time2 = time.time()

    except KeyboardInterrupt:
        print('\nKeyboard interrupt received. Shutting down...')
    finally:
        # Ensure sensors are properly disconnected
        try:
            sensor1.disconnect()
            if dual_sensor_mode and sensor2:
                sensor2.disconnect()
        except:
            pass  # Ignore errors during cleanup
        cv2.destroyAllWindows()
        print("Cleanup completed.")