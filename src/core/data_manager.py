"""
Data Manager for Gripper Control System
Handles data recording, tracking, and JSON snapshot generation
"""

import time
import threading
import json
import os
from typing import Optional, Dict, Any, List, Tuple

# Import recording configuration
from src.core.config_loader import RECORDING_CONFIG

# Try to import numpy
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: numpy not available for mesh calculations in data_manager")


class DataManager:
    """Manages data recording sessions and JSON snapshot generation"""
    
    def __init__(self):
        self.recording = False
        self.session_start_time = None
        self.session_name = ""
        self.recording_thread = None
        self.stop_recording_event = threading.Event()
        
        # Ensure data directories exist
        self._ensure_directories()
    
    def _ensure_directories(self):
        """Create necessary data directories"""
        snapshots_dir = RECORDING_CONFIG["snapshots_dir"]
        
        try:
            os.makedirs(snapshots_dir, exist_ok=True)
        except Exception as e:
            print(f"❌ Error creating data directories: {e}")
    
    def start_recording_session(self, session_name: str, system_state) -> bool:
        """
        Start a new recording session in a separate thread
        
        Args:
            session_name: Name identifier for the session
            system_state: Reference to the main system state object
            
        Returns:
            bool: True if session started successfully, False otherwise
        """
        if self.recording:
            print("❌ Recording session already in progress. Stop current session first.")
            return False
        
        try:
            self.recording = True
            self.session_start_time = time.time()
            self.session_name = session_name
            self.stop_recording_event.clear()
            
            # Auto-zero Bota sensor at session start
            self._zero_bota_sensor(system_state)
            
            # Start recording thread
            self.recording_thread = threading.Thread(
                target=self._recording_loop,
                args=(system_state,),
                name=f"DataRecording_{session_name}",
                daemon=True
            )
            self.recording_thread.start()
            
            print(f"🔴 Started recording session: {session_name}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start recording session: {e}")
            self.recording = False
            return False
    
    def stop_recording_session(self) -> bool:
        """
        Stop the current recording session
        
        Returns:
            bool: True if session stopped successfully, False otherwise
        """
        if not self.recording:
            print("⚠️ No recording session in progress")
            return False
        
        try:
            print(f"⏹️ Stopping recording session: {self.session_name}")
            
            # Signal the recording thread to stop
            self.stop_recording_event.set()
            
            # Wait for thread to finish
            if self.recording_thread and self.recording_thread.is_alive():
                self.recording_thread.join(timeout=5.0)
                if self.recording_thread.is_alive():
                    print("⚠️ Recording thread did not stop gracefully")
            
            # Reset state
            self.recording = False
            self.session_start_time = None
            self.session_name = ""
            self.recording_thread = None
            
            print("✅ Recording session stopped")
            return True
            
        except Exception as e:
            print(f"❌ Error stopping recording session: {e}")
            return False
    
    def _zero_bota_sensor(self, system_state):
        """
        Zero the Bota sensor force/torque values at the start of a recording session
        This sets the current readings as the new baseline (zero point)
        
        Args:
            system_state: Reference to the main system state object
        """
        try:
            # Try to zero the main bota sensor in sensors
            if hasattr(system_state.sensors, 'bota') and system_state.sensors.bota.connected:
                bota = system_state.sensors.bota
                if hasattr(bota, 'zero') and callable(getattr(bota, 'zero')):
                    bota.zero()
                    print("🔧 Bota sensor zeroed successfully")
                    return
                elif hasattr(bota, 'tare') and callable(getattr(bota, 'tare')):
                    bota.tare()
                    print("🔧 Bota sensor tared successfully")
                    return
                else:
                    print("⚠️ Bota sensor does not support zeroing/taring")
            
            # Fallback: try to zero bota_sensor (if it exists as separate manager)
            elif hasattr(system_state, 'bota_sensor') and system_state.bota_sensor.connected:
                bota = system_state.bota_sensor
                if hasattr(bota, 'zero') and callable(getattr(bota, 'zero')):
                    bota.zero()
                    print("🔧 Bota sensor zeroed successfully")
                    return
                elif hasattr(bota, 'tare') and callable(getattr(bota, 'tare')):
                    bota.tare()
                    print("🔧 Bota sensor tared successfully")
                    return
                else:
                    print("⚠️ Bota sensor does not support zeroing/taring")
            else:
                print("⚠️ Bota sensor not connected - skipping zeroing (normal for experiments without force sensor)")
                
        except Exception as e:
            print(f"❌ Error zeroing Bota sensor: {e}")

    def _recording_loop(self, system_state):
        """
        Main recording loop that runs in a separate thread
        Continuously captures snapshots at regular intervals
        """
        snapshot_interval = RECORDING_CONFIG["snapshot_interval"]
        last_capture_time = time.time()  # Track absolute time, not elapsed time
        
        try:
            while not self.stop_recording_event.is_set():
                current_time = time.time()
                elapsed_time = current_time - self.session_start_time
                
                # Check if it's time for a snapshot based on absolute time intervals
                if current_time - last_capture_time >= snapshot_interval:
                    success = self._capture_snapshot(system_state, elapsed_time)
                    if success:
                        last_capture_time = current_time  # Update to current absolute time
                
                # Brief sleep to prevent excessive CPU usage
                time.sleep(0.01)  # Reduced to 10ms for more precise timing
                
        except Exception as e:
            print(f"❌ Error in recording loop: {e}")
        finally:
            print(f"📊 Recording loop finished for session: {self.session_name}")
    
    def _capture_snapshot(self, system_state, elapsed_time: float) -> bool:
        """
        Capture a single snapshot of all system data
        
        Args:
            system_state: Reference to the main system state object
            elapsed_time: Time elapsed since session start in seconds
            
        Returns:
            bool: True if snapshot was captured successfully, False otherwise
        """
        try:
            # Get current timestamp
            current_timestamp = time.time()
            
            # Collect data from all system components
            snapshot_data = {
                "timestamp": current_timestamp,
                "elapsed_time_s": round(elapsed_time, 3),
                "session_name": self.session_name
            }
            
            # 1. Daimon sensor data
            daimon_data = self._collect_daimon_data(system_state)
            if daimon_data:
                snapshot_data["daimon"] = daimon_data
            
            # 2. Gripper data
            gripper_data = self._collect_gripper_data(system_state)
            if gripper_data:
                snapshot_data["gripper"] = gripper_data
            
            # 3. Bota sensor data (always include, even if null when sensor not connected)
            bota_data = self._collect_bota_data(system_state)
            snapshot_data["bota"] = bota_data  # Include even if None for consistency
            
            # 4. UR robot pose data (always include, even if null when robot not connected)
            ur_robot_data = self._collect_ur_robot_data(system_state)
            snapshot_data["ur_robot"] = ur_robot_data  # Include even if None for consistency
            
            # Save snapshot to JSON file
            return self._save_snapshot_json(snapshot_data, current_timestamp, elapsed_time)
            
        except Exception as e:
            print(f"❌ Error capturing snapshot at {elapsed_time:.1f}s: {e}")
            return False
    
    def _collect_daimon_data(self, system_state) -> Optional[Dict[str, Any]]:
        """
        Collect data from the Daimon visuotactile sensor
        
        Returns:
            dict: Daimon sensor data or None if not available
        """
        try:
            if not system_state.sensors.visuotactile.connected:
                return None
            
            sensor = system_state.sensors.visuotactile
            
            # Skip the large depth array for high-frequency data collection
            # Only collect essential depth-related data (max intensity and centerline)
            
            # Get deformation mesh data using the correct method
            deformation_mesh = None
            try:
                deform_data = sensor.get_deformation_mesh(scale=20.0, grid_n=15)
                if deform_data:
                    deformation_mesh = {
                        "start_coordinates": deform_data['start_coords'],
                        "magnitudes": deform_data['magnitudes'],
                        "directions": deform_data['directions'],
                        "grid_size": deform_data['grid_size'],
                        "scale_factor": deform_data['scale_factor']
                    }
            except Exception as e:
                print(f"Warning: Could not get deformation mesh: {e}")
            
            # Get shear mesh data using the correct method
            shear_mesh = None
            try:
                shear_data = sensor.get_shear_mesh(scale=20.0, grid_n=15)
                if shear_data:
                    shear_mesh = {
                        "start_coordinates": shear_data['start_coords'],
                        "magnitudes": shear_data['magnitudes'],
                        "directions": shear_data['directions'],
                        "grid_size": shear_data['grid_size'],
                        "scale_factor": shear_data['scale_factor']
                    }
            except Exception as e:
                print(f"Warning: Could not get shear mesh: {e}")
            
            # Get max intensity (correct attribute name)
            max_intensity = getattr(sensor, 'max_depth_intensity', 0.0)
            
            # Get centerline angle offset (correct attribute name)
            angle_offset = getattr(sensor, 'angle_offset', 0.0)
            
            # Get new sensor measurements
            # 1. Deformation integral - sum of all deformation arrow magnitudes
            deformation_integral = None
            try:
                deformation_integral = sensor.get_deformation_integral(scale=20.0, grid_n=15)
            except Exception as e:
                print(f"Warning: Could not get deformation integral: {e}")
            
            # 2. Resultant shear vector - x and y components of net shear force
            shear_vector_x = None
            shear_vector_y = None
            try:
                resultant_data = sensor.get_resultant_shear_vector(scale=20.0, grid_n=15)
                if resultant_data is not None:
                    # New format returns ((x, y), magnitude)
                    (shear_vector_x, shear_vector_y), force_magnitude = resultant_data
            except Exception as e:
                print(f"Warning: Could not get resultant shear vector: {e}")
            
            # 3. Depth intensity integral - sum of all depth pixel intensities
            depth_integral = None
            try:
                depth_integral = sensor.get_depth_intensity_integral()
            except Exception as e:
                print(f"Warning: Could not get depth intensity integral: {e}")
            
            return {
                "deformation_mesh": deformation_mesh,
                "shear_mesh": shear_mesh,
                "max_intensity": round(max_intensity, 6),
                "centerline_angle_offset": round(angle_offset, 3) if angle_offset is not None else None,
                # New scalar measurements
                "deformation_integral": round(deformation_integral, 6) if deformation_integral is not None else None,
                "shear_vector_x": round(shear_vector_x, 6) if shear_vector_x is not None else None,
                "shear_vector_y": round(shear_vector_y, 6) if shear_vector_y is not None else None,
                "depth_intensity_integral": round(depth_integral, 6) if depth_integral is not None else None
            }
            
        except Exception as e:
            print(f"❌ Error collecting Daimon data: {e}")
            return None
    
    def _collect_gripper_data(self, system_state) -> Optional[Dict[str, Any]]:
        """
        Collect data from the gripper system (DM motor + stepper motors)
        
        Returns:
            dict: Gripper system data or None if not available
        """
        try:
            # Get DM motor (gripper) data
            dm_position = None
            gripper_closure_percent = None
            if system_state.hardware.gripper.connected:
                dm_position = getattr(system_state.hardware.gripper, 'gripper_position_rad', None)
                gripper_closure_percent = getattr(system_state.hardware.gripper, 'gripper_closure_percent', None)
            
            # Get stepper motor data
            m1_position = None
            m2_position = None
            target_steps = None
            speed = None
            
            if system_state.hardware.stepper.connected:
                stepper = system_state.hardware.stepper
                m1_position = getattr(stepper, 'm1_target_pos', None)
                m2_position = getattr(stepper, 'm2_target_pos', None)
                # Get the fixed target steps from config - this is the step increment for movements
                from src.core.config_loader import CONFIG
                target_steps = CONFIG.get("fixed_target_steps", None)
                speed = getattr(stepper, 'speed', None)
            
            return {
                "dm_motor_position_rad": round(dm_position, 6) if dm_position is not None else None,
                "gripper_closure_percent": round(gripper_closure_percent, 2) if gripper_closure_percent is not None else None,
                "m1_stepper_position": m1_position,
                "m2_stepper_position": m2_position,
                "step_size": target_steps,  # This now tracks the target_steps from config
                "speed_steps_per_sec": speed
            }
            
        except Exception as e:
            print(f"❌ Error collecting gripper data: {e}")
            return None
    
    def _collect_bota_data(self, system_state) -> Optional[Dict[str, Any]]:
        """
        Collect 6-DOF force and torque data from Bota sensor
        
        Returns:
            dict: Bota sensor data with null values if sensor not connected, or None if error
        """
        try:
            # Try to get data from the main bota sensor in sensors
            if hasattr(system_state.sensors, 'bota') and system_state.sensors.bota.connected:
                bota = system_state.sensors.bota
                latest_data = getattr(bota, 'latest_data', {})
                
                return {
                    "force_x_N": round(latest_data.get('Fx', 0.0), 6),
                    "force_y_N": round(latest_data.get('Fy', 0.0), 6),
                    "force_z_N": round(latest_data.get('Fz', 0.0), 6),
                    "torque_x_Nm": round(latest_data.get('Mx', 0.0), 6),
                    "torque_y_Nm": round(latest_data.get('My', 0.0), 6),
                    "torque_z_Nm": round(latest_data.get('Mz', 0.0), 6)
                }
            
            # Fallback: try to get data from bota_sensor (if it exists as separate manager)
            elif hasattr(system_state, 'bota_sensor') and system_state.bota_sensor.connected:
                bota = system_state.bota_sensor
                latest_data = getattr(bota, 'latest_data', {})
                
                return {
                    "force_x_N": round(latest_data.get('Fx', 0.0), 6),
                    "force_y_N": round(latest_data.get('Fy', 0.0), 6),
                    "force_z_N": round(latest_data.get('Fz', 0.0), 6),
                    "torque_x_Nm": round(latest_data.get('Mx', 0.0), 6),
                    "torque_y_Nm": round(latest_data.get('My', 0.0), 6),
                    "torque_z_Nm": round(latest_data.get('Mz', 0.0), 6)
                }
            
            # Sensor not connected - return null values for experiments without force sensor
            else:
                return {
                    "force_x_N": None,
                    "force_y_N": None,
                    "force_z_N": None,
                    "torque_x_Nm": None,
                    "torque_y_Nm": None,
                    "torque_z_Nm": None
                }
            
        except Exception as e:
            print(f"❌ Error collecting Bota data: {e}")
            return None
    
    def _collect_ur_robot_data(self, system_state) -> Optional[Dict[str, Any]]:
        """
        Collect 6-DOF pose data from UR robot (position + orientation)
        
        Returns:
            dict: UR robot pose data with null values if robot not connected, or None if error
        """
        try:
            # Check if UR robot is connected and has current pose data
            if system_state.hardware.ur_robot.connected and system_state.hardware.ur_robot.current_pose is not None:
                pose = system_state.hardware.ur_robot.current_pose
                
                # UR robot pose format: [x, y, z, rx, ry, rz]
                # x, y, z are position in meters
                # rx, ry, rz are rotation vector (axis-angle representation)
                return {
                    "position_x_m": round(pose[0], 6),
                    "position_y_m": round(pose[1], 6),
                    "position_z_m": round(pose[2], 6),
                    "rotation_x_rad": round(pose[3], 6),
                    "rotation_y_rad": round(pose[4], 6),
                    "rotation_z_rad": round(pose[5], 6)
                }
            
            # Robot not connected or no pose data - return null values
            else:
                return {
                    "position_x_m": None,
                    "position_y_m": None,
                    "position_z_m": None,
                    "rotation_x_rad": None,
                    "rotation_y_rad": None,
                    "rotation_z_rad": None
                }
            
        except Exception as e:
            print(f"❌ Error collecting UR robot data: {e}")
            return None
    
    def _save_snapshot_json(self, snapshot_data: Dict[str, Any], timestamp: float, elapsed_time: float) -> bool:
        """
        Save snapshot data to a JSON file
        
        Args:
            snapshot_data: Complete snapshot data dictionary
            timestamp: Absolute timestamp
            elapsed_time: Time elapsed since session start
            
        Returns:
            bool: True if saved successfully, False otherwise
        """
        try:
            # Create snapshots directory path
            snapshots_dir = RECORDING_CONFIG["snapshots_dir"]
            
            # Create session subdirectory if enabled
            if RECORDING_CONFIG["create_session_subdirectories"]:
                snapshots_dir = os.path.join(snapshots_dir, self.session_name)
                os.makedirs(snapshots_dir, exist_ok=True)
            
            # Create filename with session name, timestamp and elapsed time
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(timestamp))
            filename = f"{self.session_name}_{timestamp_str}_{elapsed_time:.1f}s.json"
            
            file_path = os.path.join(snapshots_dir, filename)
            
            # Save to JSON file with proper formatting
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(snapshot_data, f, indent=2, default=str)
            
            print(f"📸 Snapshot saved: {elapsed_time:.1f}s - {filename}")
            return True
            
        except Exception as e:
            print(f"❌ Error saving snapshot JSON: {e}")
            return False
    
    def is_recording(self) -> bool:
        """Check if a recording session is currently active"""
        return self.recording
    
    def get_session_info(self) -> Dict[str, Any]:
        """Get information about the current recording session"""
        if not self.recording:
            return {"status": "idle"}
        
        elapsed_time = time.time() - self.session_start_time if self.session_start_time else 0
        
        return {
            "status": "recording",
            "session_name": self.session_name,
            "start_time": self.session_start_time,
            "elapsed_time_s": round(elapsed_time, 1)
        }
