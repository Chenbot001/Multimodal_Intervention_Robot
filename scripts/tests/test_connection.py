#!/usr/bin/env python3
"""
Test script to verify the fixes for the modular gripper system.
Tests:
1. Sensor image display
2. Encoder recording logic
"""

import time
import sys
import os

# Add the current directory to the path
sys.path.append(os.path.dirname(__file__))

from src.core.config_loader import DISPLAY_CONFIG
from src.core.sensor_manager import SensorManager
from src.core.hardware_manager import HardwareManager

def test_sensor_images():
    """Test sensor image display"""
    print("Testing sensor image display...")
    
    # Create sensor manager
    sensors = SensorManager()
    
    # Initialize sensors
    status = sensors.initialize()
    print(f"Sensor initialization status: {status}")
    
    if status['visuotactile']:
        print("✓ Visuotactile sensor connected")
        
        # Test image display for a few seconds
        print("Displaying sensor images for 5 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 5:
            if sensors.visuotactile.sensor:
                try:
                    sensors.visuotactile.display_sensor_images()
                    time.sleep(0.1)
                except Exception as e:
                    print(f"Error displaying images: {e}")
                    break
        
        print("✓ Sensor image display test completed")
    else:
        print("⚠ Visuotactile sensor not available")
    
    # Cleanup
    sensors.cleanup()

def test_encoder_recording():
    """Test encoder recording logic"""
    print("\nTesting encoder recording logic...")
    
    # Create sensor manager
    sensors = SensorManager()
    
    # Initialize sensors
    status = sensors.initialize()
    print(f"Sensor initialization status: {status}")
    
    if status['encoder']:
        print("✓ Encoder connected")
        
        # Test metadata update
        sensors.encoder.update_recording_metadata(2.0, 90.0, 0.5, 0.0)  # Added tilt parameter
        print(f"Recording metadata: diameter={sensors.encoder.recording_diameter}, "
              f"target={sensors.encoder.recording_target_angle}, "
              f"grip_strength={sensors.encoder.recording_grip_strength}, "
              f"tilt={sensors.encoder.recording_tilt}")
        
        # Test recording start (should fail since gripper is open)
        sensors.encoder.start_encoder_recording('cw', 0.0, 0.0)
        print("✓ Encoder recording logic test completed")
    else:
        print("⚠ Encoder not available")
    
    # Cleanup
    sensors.cleanup()

def test_configuration():
    """Test configuration settings"""
    print("\nTesting configuration...")
    
    print(f"Show sensor images: {DISPLAY_CONFIG['show_sensor_images']}")
    print(f"Sensor image update rate: {DISPLAY_CONFIG['sensor_image_update_rate_hz']} Hz")
    
    # Test disabling sensor images
    DISPLAY_CONFIG['show_sensor_images'] = False
    print("✓ Configuration test completed")

if __name__ == "__main__":
    print("=" * 60)
    print("           GRIPPER SYSTEM FIXES TEST")
    print("=" * 60)
    
    try:
        test_configuration()
        test_sensor_images()
        test_encoder_recording()
        
        print("\n" + "=" * 60)
        print("           ALL TESTS COMPLETED")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
