"""
Sensor Manager for the Gripper Control System
Manages visuotactile sensor interactions
"""

import time
import threading
import sys
import os

# Try to import numpy, but don't fail if it's not available
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: numpy not available. Some sensor functions will be disabled.")

# Try to import cv2, but don't fail if it's not available
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: cv2 not available. Image display functions will be disabled.")

# Try to import Bota sensor dependencies
try:
    import pysoem
    import ctypes
    import struct
    import collections
    BOTA_AVAILABLE = True
except ImportError:
    BOTA_AVAILABLE = False
    print("Warning: Bota sensor dependencies not available. Force sensing disabled.")

# Add paths for local imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'sensors', 'daimon'))

from src.core.config_loader import (
    CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, CENTERLINE_CONFIG, SAFETY_CONFIG
)

# Import sensor functionality
try:
    from dmrobotics import Sensor, put_arrows_on_image
    SENSOR_AVAILABLE = True
except ImportError:
    print("Warning: Sensor library not available. Depth sensing disabled.")
    SENSOR_AVAILABLE = False


class DaimonManager:
    """Manages Daimon visuotactile sensor operations"""
    
    def __init__(self):
        self.sensor = None
        self.connected = False
        self.last_message = ""
        self.max_depth_intensity = 0.0
        self.baseline_intensity = 0.0
        self.net_intensity = 0.0
        self.sensor_fps = 0.0
        self.frame_count = 0
        self.fps_start_time = 0.0
        
        # Centerline detection variables
        self.centerline_points = None
        self.largest_contour = None
        self.angle_offset = None
        self.contact_threshold = 0.25
        
        # Centerline smoothing and persistence variables
        self.last_valid_angle_offset = None
        self.angle_history = []
        self.max_history_length = CENTERLINE_CONFIG["history_length"]
        self.smoothing_alpha = CENTERLINE_CONFIG["smoothing_alpha"]
        
        # Safety monitoring variables
        self.last_safety_trigger_time = 0.0
        self.safety_callback = None  # Callback function for emergency gripper opening
        self.last_shear_magnitude = 0.0
        self.last_shear_x = 0.0      # Last shear force X component
        self.last_shear_y = 0.0      # Last shear force Y component
        self.safety_triggered = False
        self.safety_trigger_reason = ""  # Reason for last safety trigger
        
        # Baseline shear force components for zeroing
        self.baseline_shear_x = 0.0
        self.baseline_shear_y = 0.0

    def connect(self):
        """Initialize and configure the depth sensor"""
        if not SENSOR_AVAILABLE:
            self.last_message = "Sensor library not available"
            return False
        
        try:
            self.sensor = Sensor(CONFIG["sensor_serial_id"])
            self.connected = True
            self.last_message = "Sensor connected successfully"
            print("✓ Depth sensor connected")
            return True
        except Exception as e:
            self.last_message = f"ERR: Sensor setup failed: {e}"
            print("❌ Depth sensor setup failed")
            return False
    
    def disconnect(self):
        """Disconnect sensor and close windows"""
        if self.sensor and self.connected:
            try:
                self.close_sensor_windows()
                self.sensor.disconnect()
                print("✓ Sensor disconnected")
            except:
                pass
        self.connected = False
    
    def get_max_depth_intensity(self):
        """Get the sensor depth image and return the maximum intensity"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return 0.0
        
        if not NUMPY_AVAILABLE:
            self.last_message = "Numpy not available for depth processing"
            return 0.0
        
        try:
            depth = self.sensor.getDepth()
            
            if depth is not None and depth.size > 0:
                max_intensity = float(np.max(depth))
                self.max_depth_intensity = max_intensity
                self.net_intensity = max_intensity - self.baseline_intensity
                self.last_message = f"Max: {max_intensity:.3f}, Net: {self.net_intensity:.3f}"
                return max_intensity
            else:
                self.last_message = "No depth data available"
                return 0.0
                
        except Exception as e:
            self.last_message = f"ERR: Failed to get depth intensity: {e}"
            return 0.0
    
    def get_depth_intensity_integral(self):
        """
        Calculate the depth intensity integral by summing all pixel intensities in the depth image.
        This provides a measure of the total contact area and intensity across the entire sensor surface.
        
        Returns:
            float: Sum of all depth pixel intensities, or 0.0 if unavailable
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            return 0.0
        
        if not NUMPY_AVAILABLE:
            self.last_message = "Numpy not available for depth integral processing"
            return 0.0
        
        try:
            depth = self.sensor.getDepth()
            
            if depth is not None and depth.size > 0:
                # Calculate the integral (sum of all pixel intensities)
                depth_integral = float(np.sum(depth))
                
                # Also calculate net integral (removing baseline contribution)
                baseline_contribution = self.baseline_intensity * depth.size
                net_depth_integral = depth_integral - baseline_contribution
                
                self.last_message = f"Depth integral: {depth_integral:.3f}, Net integral: {net_depth_integral:.3f}"
                return depth_integral
            else:
                self.last_message = "No depth data available for integral calculation"
                return 0.0
                
        except Exception as e:
            self.last_message = f"ERR: Failed to calculate depth integral: {e}"
            return 0.0
    
    def calibrate_baseline_intensity(self, samples=10):
        """Calibrate the baseline intensity when gripper is fully open"""
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for calibration"
            return False
        
        try:
            total_intensity = 0.0
            valid_samples = 0
            
            for i in range(samples):
                intensity = self.get_max_depth_intensity()
                if intensity > 0.0:
                    total_intensity += intensity
                    valid_samples += 1
                time.sleep(0.1)
            
            if valid_samples > 0:
                self.baseline_intensity = total_intensity / valid_samples
                self.net_intensity = 0.0
                self.last_message = f"Baseline calibrated: {self.baseline_intensity:.3f}"
                print(f"✓ Baseline intensity calibrated: {self.baseline_intensity:.3f}")
                return True
            else:
                self.last_message = "ERR: No valid samples for calibration"
                return False
                
        except Exception as e:
            self.last_message = f"ERR: Calibration failed: {e}"
            return False
    
    def calibrate_baseline_shear_force(self, samples=10):
        """
        Calibrate the baseline shear force components when gripper is fully open and no object present.
        This sets the current shear readings as the new baseline (zero point).
        
        Args:
            samples (int): Number of samples to average for baseline calculation
            
        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for shear force calibration"
            return False
        
        try:
            total_shear_x = 0.0
            total_shear_y = 0.0
            valid_samples = 0
            
            for i in range(samples):
                # Get current shear force components
                resultant_data = self.get_resultant_shear_vector(scale=20.0, grid_n=15)
                if resultant_data is not None:
                    (shear_x, shear_y), force_magnitude = resultant_data
                    total_shear_x += shear_x
                    total_shear_y += shear_y
                    valid_samples += 1
                time.sleep(0.1)
            
            if valid_samples > 0:
                self.baseline_shear_x = total_shear_x / valid_samples
                self.baseline_shear_y = total_shear_y / valid_samples
                self.last_message = f"Shear baseline calibrated: X={self.baseline_shear_x:.4f}N, Y={self.baseline_shear_y:.4f}N"
                print(f"✓ Baseline shear force calibrated: X={self.baseline_shear_x:.4f}N, Y={self.baseline_shear_y:.4f}N")
                return True
            else:
                self.last_message = "ERR: No valid samples for shear force calibration"
                return False
                
        except Exception as e:
            self.last_message = f"ERR: Shear force calibration failed: {e}"
            return False
    
    def compute_centerline_angle(self, centerline_points, state=None):
        """
        Compute the angle offset of the centerline from vertical (0 degrees baseline) with smoothing.
        
        Args:
            centerline_points: Array of points along the centerline
            state: System state object to check gripper status
            
        Returns:
            angle_offset: Smoothed angle offset from vertical in degrees, last valid angle if gripping, or None if open and no detection
        """
        if not NUMPY_AVAILABLE or centerline_points is None or len(centerline_points) < 2:
            # Failed detection - behavior depends on gripper state
            if state and hasattr(state, 'hardware') and hasattr(state.hardware, 'gripper'):
                gripper = state.hardware.gripper
                
                # If gripper is actively gripping or closed significantly, hold last valid angle
                if gripper.is_gripping or gripper.gripper_closure_percent > CONFIG["gripper_default_open_percent"]:
                    # Gripper is closed/gripping - hold the last valid angle
                    return self.last_valid_angle_offset
                else:
                    # Gripper is open - reset to None (N/A) since no object should be present
                    return None
            else:
                # Fallback to old behavior if state is not available
                return self.last_valid_angle_offset
        
        # Use first and last points to determine the overall line direction
        start_point = centerline_points[0]
        end_point = centerline_points[-1]
        
        # Calculate direction vector
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        # Calculate angle from vertical (negative y-axis is up in image coordinates)
        # atan2 gives angle from positive x-axis, we want angle from negative y-axis (vertical up)
        angle_from_horizontal = np.arctan2(dy, dx)  # Angle from positive x-axis
        angle_from_vertical = angle_from_horizontal - np.pi/2  # Convert to angle from vertical
        
        # Convert to degrees
        angle_degrees = np.degrees(angle_from_vertical)
        
        # Normalize to [-90, 90] range and take absolute value
        # This ensures both CW and CCW deviations are positive
        if angle_degrees > 90:
            angle_degrees = 180 - angle_degrees
        elif angle_degrees < -90:
            angle_degrees = -180 - angle_degrees
        
        # Take absolute value for positive offset regardless of direction
        raw_angle = abs(angle_degrees)
        
        # Apply smoothing filter
        smoothed_angle = self._apply_angle_smoothing(raw_angle)
        
        # Update last valid angle
        self.last_valid_angle_offset = smoothed_angle
        
        return smoothed_angle
    
    def _apply_angle_smoothing(self, new_angle):
        """
        Apply smoothing filter to reduce angle fluctuations.
        Uses both exponential smoothing and rolling average.
        
        Args:
            new_angle: New raw angle measurement
            
        Returns:
            smoothed_angle: Filtered angle value
        """
        if not NUMPY_AVAILABLE:
            return new_angle
        
        # Add to history buffer
        self.angle_history.append(new_angle)
        
        # Keep history buffer size limited
        if len(self.angle_history) > self.max_history_length:
            self.angle_history.pop(0)
        
        # Calculate rolling average
        rolling_average = np.mean(self.angle_history)
        
        # Apply exponential smoothing with previous valid angle if available
        if self.last_valid_angle_offset is not None:
            # Exponential smoothing: new_value = alpha * old_value + (1 - alpha) * new_value
            smoothed_angle = (self.smoothing_alpha * self.last_valid_angle_offset + 
                            (1 - self.smoothing_alpha) * rolling_average)
        else:
            # No previous data, use rolling average
            smoothed_angle = rolling_average
        
        return smoothed_angle
    
    def reset_centerline_smoothing(self):
        """
        Reset the centerline smoothing history and persistent values.
        Call this when recalibrating or when you want to start fresh.
        """
        self.angle_history.clear()
        self.last_valid_angle_offset = None
        self.last_message = "Centerline smoothing reset"
    
    def detect_centerline(self, depth_data):
        """
        Detect centerline from depth data by finding contours above threshold and fitting a line.
        
        Args:
            depth_data: The raw depth data array
            
        Returns:
            centerline_points: Array of points along the centerline, or None if no centerline found
            largest_contour: The largest contour found, or None
        """
        if not NUMPY_AVAILABLE or not CV2_AVAILABLE or depth_data is None:
            return None, None
        
        try:
            # Create binary mask for pixels above threshold
            contact_mask = (depth_data > self.contact_threshold).astype(np.uint8) * 255
            
            # Apply morphological operations to reduce noise
            kernel = np.ones((3,3), np.uint8)
            contact_mask = cv2.morphologyEx(contact_mask, cv2.MORPH_CLOSE, kernel)
            contact_mask = cv2.morphologyEx(contact_mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(contact_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                return None, None
            
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            
            # More lenient area threshold to prevent disappearing centerlines
            min_area_threshold = CENTERLINE_CONFIG["min_contour_area"]
            if cv2.contourArea(largest_contour) < min_area_threshold:
                return None, None
            
            # Fit a line to the contour points
            # Reshape contour points for cv2.fitLine
            contour_points = largest_contour.reshape(-1, 2)
            
            if len(contour_points) < 5:  # Need minimum points for line fitting
                return None, None
            
            # Fit line using least squares
            [vx, vy, x0, y0] = cv2.fitLine(contour_points, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # Generate centerline points along the fitted line
            # Get bounding box of the contour to determine line extent
            x_min, y_min, w, h = cv2.boundingRect(largest_contour)
            x_max = x_min + w
            y_max = y_min + h
            
            # Calculate line parameters
            if abs(vx) > 1e-6:  # Avoid division by zero
                # Calculate t values for line boundaries
                t1 = (x_min - x0) / vx
                t2 = (x_max - x0) / vx
                t_min = min(t1, t2)
                t_max = max(t1, t2)
                
                # Generate points along the line
                t_values = np.linspace(t_min, t_max, 50)
                centerline_points = np.column_stack([
                    x0 + t_values * vx,
                    y0 + t_values * vy
                ])
            else:
                # Vertical line case
                centerline_points = np.column_stack([
                    np.full(50, x0),
                    np.linspace(y_min, y_max, 50)
                ])
            
            # Filter points to stay within image bounds
            h_img, w_img = depth_data.shape
            valid_mask = (
                (centerline_points[:, 0] >= 0) & 
                (centerline_points[:, 0] < w_img) &
                (centerline_points[:, 1] >= 0) & 
                (centerline_points[:, 1] < h_img)
            )
            centerline_points = centerline_points[valid_mask]
            
            if len(centerline_points) < 2:
                return None, None
            
            return centerline_points.astype(int), largest_contour
            
        except Exception as e:
            self.last_message = f"ERR: Centerline detection failed: {e}"
            return None, None
    
    def add_centerline_overlay(self, depth_img, centerline_points, largest_contour=None):
        """
        Add centerline and contour overlay to the depth image.
        
        Args:
            depth_img: The depth image (BGR format)
            centerline_points: Array of points along the centerline
            largest_contour: The contour from which centerline was derived
            
        Returns:
            The depth image with centerline overlaid
        """
        if not CV2_AVAILABLE or depth_img is None:
            return depth_img
        
        depth_img_with_centerline = depth_img.copy()
        
        if centerline_points is not None and len(centerline_points) > 1:
            # Draw the centerline (50% thinner: 3 -> 1.5, rounded to 2)
            for i in range(len(centerline_points) - 1):
                pt1 = tuple(centerline_points[i])
                pt2 = tuple(centerline_points[i + 1])
                cv2.line(depth_img_with_centerline, pt1, pt2, (0, 255, 0), 2)  # Green line
            
            # Optionally draw the contour outline (50% thinner: 2 -> 1)
            if largest_contour is not None:
                cv2.drawContours(depth_img_with_centerline, [largest_contour], -1, (0, 255, 255), 1)  # Yellow contour
        
        return depth_img_with_centerline
    
    def add_angle_offset_overlay(self, depth_img, angle_offset, centerline_detected):
        """
        Add angle offset value overlay to the depth image.
        
        Args:
            depth_img: The colorized depth image (BGR format)
            angle_offset: The angle offset from vertical in degrees, or None if no centerline
            centerline_detected: Whether centerline was detected in current frame
            
        Returns:
            The depth image with angle offset value overlaid
        """
        if not CV2_AVAILABLE or depth_img is None:
            return depth_img
        
        # Create a copy of the depth image to avoid modifying the original
        depth_img_with_overlay = depth_img.copy()
        
        # Prepare the text to display
        if angle_offset is not None:
            # Round to nearest 10 degrees for better readability - remove for exact angle
            rounded_angle = round(angle_offset / 10) * 10
            # Add indicator if using persistent angle vs live detection
            status_indicator = "Live" if centerline_detected else "Hold"
            angle_text = f"Angle: {rounded_angle:.0f} degrees ({status_indicator})"
        else:
            # angle_offset is None - this means gripper is open and no detection (N/A case)
            angle_text = "Angle: N/A (Open)"
        
        # Set text properties - use green color for all angle displays
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        # Always use green color regardless of live vs persistent status
        color = (0, 255, 0)  # Green in BGR format
        thickness = 1
        
        # Get text size to position it properly
        (text_width, text_height), baseline = cv2.getTextSize(angle_text, font, font_scale, thickness)
        
        # Position the text in the top-left corner with some padding
        x = 10
        y = text_height + 15
        
        # Add a semi-transparent background rectangle for better readability
        overlay = depth_img_with_overlay.copy()
        cv2.rectangle(overlay, (x-5, y-text_height-5), (x+text_width+5, y+baseline+5), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, depth_img_with_overlay, 0.4, 0, depth_img_with_overlay)
        
        # Add the text
        cv2.putText(depth_img_with_overlay, angle_text, (x, y), font, font_scale, color, thickness)
        
        return depth_img_with_overlay
    
    def add_resultant_shear_overlay(self, shear_img, resultant_vector, resultant_force_magnitude):
        """
        Add resultant shear vector overlay to the shear image.
        
        Args:
            shear_img: The shear image (BGR format)
            resultant_vector: Tuple (shear_x, shear_y) of resultant shear force components
            resultant_force_magnitude: The magnitude of the resultant shear force (not displayed)
            
        Returns:
            The shear image with resultant force X and Y component values overlaid
        """
        if not CV2_AVAILABLE or shear_img is None or resultant_vector is None:
            return shear_img
        
        # Create a copy of the shear image to avoid modifying the original
        shear_img_with_overlay = shear_img.copy()
        
        # Get image dimensions
        height, width = shear_img.shape[:2]
        
        # Get shear force components
        shear_x, shear_y = resultant_vector
        
        # Add text showing the shear force X and Y components
        force_x_text = f"Force X: {shear_x:.3f}N"
        force_y_text = f"Force Y: {shear_y:.3f}N"
        
        # Set text properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        color = (0, 255, 255)  # Cyan color
        thickness = 1
        
        # Get text sizes for positioning
        (x_text_width, x_text_height), _ = cv2.getTextSize(force_x_text, font, font_scale, thickness)
        (y_text_width, y_text_height), _ = cv2.getTextSize(force_y_text, font, font_scale, thickness)
        
        # Position text in top-right corner
        x_x = width - x_text_width - 10
        x_y = x_text_height + 15
        y_x = width - y_text_width - 10
        y_y = x_text_height + y_text_height + 25
        
        # Add semi-transparent background rectangles for better readability
        overlay = shear_img_with_overlay.copy()
        cv2.rectangle(overlay, (x_x-5, x_y-x_text_height-5), (x_x+x_text_width+5, x_y+5), (0, 0, 0), -1)
        cv2.rectangle(overlay, (y_x-5, y_y-y_text_height-5), (y_x+y_text_width+5, y_y+5), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, shear_img_with_overlay, 0.4, 0, shear_img_with_overlay)
        
        # Add the text
        cv2.putText(shear_img_with_overlay, force_x_text, (x_x, x_y), font, font_scale, color, thickness)
        cv2.putText(shear_img_with_overlay, force_y_text, (y_x, y_y), font, font_scale, color, thickness)
        
        return shear_img_with_overlay
    
    def set_safety_callback(self, callback_function):
        """
        Set the callback function for emergency gripper opening.
        
        Args:
            callback_function: Function to call when safety threshold is exceeded.
                              Should accept no parameters and trigger gripper opening.
        """
        self.safety_callback = callback_function
        self.last_message = "Safety callback function registered"
    
    def check_shear_safety(self, shear_force_x, shear_force_y, shear_magnitude):
        """
        Check if shear force components exceed individual safety thresholds and trigger emergency response.
        
        Args:
            shear_force_x (float): Current shear force X component in Newtons
            shear_force_y (float): Current shear force Y component in Newtons
            shear_magnitude (float): Current shear force magnitude for logging
            
        Returns:
            bool: True if safety action was triggered, False otherwise
        """
        if not SAFETY_CONFIG["safety_check_enabled"]:
            return False
        
        current_time = time.time()
        
        # Update tracking variables
        self.last_shear_x = abs(shear_force_x)  # Use absolute values for comparison
        self.last_shear_y = abs(shear_force_y)
        self.last_shear_magnitude = shear_magnitude
        
        # Check if we're in cooldown period
        if (current_time - self.last_safety_trigger_time) < SAFETY_CONFIG["safety_cooldown_seconds"]:
            return False
        
        # Check individual component thresholds
        x_threshold = SAFETY_CONFIG["shear_x_threshold_n"]
        y_threshold = SAFETY_CONFIG["shear_y_threshold_n"]
        
        x_exceeded = self.last_shear_x > x_threshold
        y_exceeded = self.last_shear_y > y_threshold
        
        # Trigger if any threshold is exceeded
        if x_exceeded or y_exceeded:
            self.last_safety_trigger_time = current_time
            self.safety_triggered = True
            
            # Determine trigger reason
            if x_exceeded and y_exceeded:
                self.safety_trigger_reason = f"Both X ({self.last_shear_x:.3f}N > {x_threshold}N) and Y ({self.last_shear_y:.3f}N > {y_threshold}N)"
            elif x_exceeded:
                self.safety_trigger_reason = f"X-axis ({self.last_shear_x:.3f}N > {x_threshold}N)"
            else:
                self.safety_trigger_reason = f"Y-axis ({self.last_shear_y:.3f}N > {y_threshold}N)"
            
            # Trigger emergency gripper opening if callback is available
            if self.safety_callback:
                try:
                    self.safety_callback()
                    self.last_message = f"🚨 SAFETY: {self.safety_trigger_reason} - Gripper opened!"
                    print(f"🚨 SAFETY TRIGGERED: Excessive shear force detected - {self.safety_trigger_reason} - Opening gripper to {SAFETY_CONFIG['emergency_open_percent']}%")
                    return True
                except Exception as e:
                    self.last_message = f"🚨 SAFETY: Shear threshold exceeded but gripper opening failed: {e}"
                    print(f"❌ Safety callback failed: {e}")
            else:
                self.last_message = f"🚨 SAFETY: {self.safety_trigger_reason} but no callback registered"
                print(f"⚠️ Safety threshold exceeded but no callback registered")
        else:
            # Reset safety trigger flag if we're below both thresholds with hysteresis
            if self.safety_triggered:
                x_reset = self.last_shear_x < (x_threshold * 0.8)  # 20% hysteresis
                y_reset = self.last_shear_y < (y_threshold * 0.8)  # 20% hysteresis
                if x_reset and y_reset:
                    self.safety_triggered = False
                    self.safety_trigger_reason = ""
        
        return False
    
    def get_safety_status(self):
        """
        Get current safety monitoring status.
        
        Returns:
            dict: Safety status information
        """
        return {
            'safety_enabled': SAFETY_CONFIG["safety_check_enabled"],
            'shear_x_threshold': SAFETY_CONFIG["shear_x_threshold_n"],
            'shear_y_threshold': SAFETY_CONFIG["shear_y_threshold_n"], 
            'shear_threshold': SAFETY_CONFIG["shear_force_threshold_n"],  # Deprecated - for backward compatibility
            'last_shear_x': self.last_shear_x,
            'last_shear_y': self.last_shear_y,
            'last_shear_magnitude': self.last_shear_magnitude,
            'baseline_shear_x': self.baseline_shear_x,
            'baseline_shear_y': self.baseline_shear_y,
            'safety_triggered': self.safety_triggered,
            'safety_trigger_reason': self.safety_trigger_reason,
            'time_since_last_trigger': time.time() - self.last_safety_trigger_time,
            'callback_registered': self.safety_callback is not None
        }
    
    def set_contact_threshold(self, threshold):
        """
        Set the contact threshold for centerline detection.
        
        Args:
            threshold: New threshold value (default: 0.15)
        """
        self.contact_threshold = threshold
        self.last_message = f"Contact threshold set to {threshold:.3f}"
    
    def get_centerline_info(self):
        """
        Get current centerline detection information.
        
        Returns:
            dict: Information about current centerline detection
        """
        return {
            'centerline_detected': self.centerline_points is not None,
            'angle_offset': self.angle_offset,
            'contact_threshold': self.contact_threshold,
            'contour_area': cv2.contourArea(self.largest_contour) if self.largest_contour is not None and CV2_AVAILABLE else 0,
            'centerline_points_count': len(self.centerline_points) if self.centerline_points is not None else 0
        }
    
    def display_sensor_images(self, state=None):
        """Display concatenated sensor images with centerline detection"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return
        
        if not CV2_AVAILABLE or not NUMPY_AVAILABLE:
            return
        
        # Check if sensor image display is enabled in configuration
        if not DISPLAY_CONFIG.get("show_sensor_images", True):
            return
        
        try:
            # Get sensor data
            img = self.sensor.getRawImage()
            depth = self.sensor.getDepth()
            deformation = self.sensor.getDeformation2D()
            shear = self.sensor.getShear()
            
            # Create depth image with color mapping
            depth_img = cv2.applyColorMap((depth * 0.25 * 255.0).astype('uint8'), cv2.COLORMAP_HOT)
            
            # Detect centerline from depth data
            self.centerline_points, self.largest_contour = self.detect_centerline(depth)
            
            # Track if centerline was detected in current frame
            centerline_detected_this_frame = self.centerline_points is not None
            
            # Compute angle offset from vertical (with smarted persistence based on gripper state)
            self.angle_offset = self.compute_centerline_angle(self.centerline_points, state)
            
            # Add centerline overlay if detected
            if self.centerline_points is not None:
                depth_img = self.add_centerline_overlay(depth_img, self.centerline_points, self.largest_contour)
            
            # Add angle offset overlay to the depth image (pass detection status for color coding)
            depth_img = self.add_angle_offset_overlay(depth_img, self.angle_offset, centerline_detected_this_frame)
            
            # Create black background for deformation and shear arrows
            black_img = np.zeros_like(img)
            black_img = np.stack([black_img] * 3, axis=-1)
            
            # Create deformation and shear images
            deformation_img = put_arrows_on_image(black_img.copy(), deformation * 20)
            shear_img = put_arrows_on_image(black_img.copy(), shear * 20)
            
            # Calculate and add resultant shear vector overlay
            try:
                resultant_data = self.get_resultant_shear_vector(scale=20.0, grid_n=15)
                if resultant_data is not None:
                    resultant_vector, resultant_force_magnitude = resultant_data
                    shear_img = self.add_resultant_shear_overlay(shear_img, resultant_vector, resultant_force_magnitude)
                    
                    # Safety check: Monitor individual shear force components
                    shear_force_x, shear_force_y = resultant_vector
                    self.check_shear_safety(shear_force_x, shear_force_y, resultant_force_magnitude)
                    
            except Exception as e:
                # Continue even if resultant shear calculation fails
                self.last_message = f"WARN: Resultant shear overlay failed: {e}"
            
            # Ensure all images have the same height for concatenation
            height = depth_img.shape[0]
            width = depth_img.shape[1]
            
            # Resize if needed
            if deformation_img.shape[:2] != (height, width):
                deformation_img = cv2.resize(deformation_img, (width, height))
            if shear_img.shape[:2] != (height, width):
                shear_img = cv2.resize(shear_img, (width, height))
            
            # Add text labels
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_color = (255, 255, 255)
            thickness = 1
            
            cv2.putText(depth_img, 'Depth', (10, height-10), font, font_scale, font_color, thickness)
            cv2.putText(deformation_img, 'Deformation', (10, height-10), font, font_scale, font_color, thickness)
            cv2.putText(shear_img, 'Shear', (10, height-10), font, font_scale, font_color, thickness)
            
            # Concatenate images horizontally
            combined_img = np.hstack([depth_img, deformation_img, shear_img])
            
            # Display the combined image
            cv2.imshow('Sensor Data - Depth | Deformation | Shear', combined_img)
            
            # Update FPS calculation
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.fps_start_time > DISPLAY_CONFIG["sensor_fps_calculation_interval"]:
                self.sensor_fps = self.frame_count / (current_time - self.fps_start_time)
                self.frame_count = 0
                self.fps_start_time = current_time
            
            # Check for window close events
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                self.sensor.reset()
                self.last_message = "Sensor reset via image window"
            
        except Exception as e:
            self.last_message = f"ERR: Image display failed: {e}"
    
    def close_sensor_windows(self):
        """Close the sensor image window"""
        if not CV2_AVAILABLE:
            return
        try:
            cv2.destroyWindow('Sensor Data - Depth | Deformation | Shear')
        except:
            pass
    
    def start_reader_thread(self, state):
        """Start background thread for reading sensor data"""
        if self.sensor and self.connected:
            self.fps_start_time = time.time()
            self.frame_count = 0
            
            reader = threading.Thread(target=self._reader_loop, args=(state,))
            reader.daemon = True
            reader.start()
            return reader
        return None
    
    def _reader_loop(self, state):
        """Background thread for reading sensor data and displaying images"""
        # Calculate sleep time based on configured update rate
        sensor_update_interval = 1.0 / DISPLAY_CONFIG.get("sensor_image_update_rate_hz", 10)
        
        while state.running:
            try:
                if self.sensor and self.connected:
                    self.get_max_depth_intensity()
                    self.display_sensor_images(state)
                    
                    # Read more frequently when gripping is active
                    if hasattr(state, 'hardware') and hasattr(state.hardware, 'gripper') and state.hardware.gripper.is_gripping:
                        time.sleep(0.01)  # 100Hz when gripping
                    else:
                        time.sleep(sensor_update_interval)  # Use configured update rate
            except Exception as e:
                self.last_message = f"ERR: Sensor reading failed: {e}"
                self.connected = False
                break
        
        self.close_sensor_windows()

    def get_latest_deformation(self):
        """Get the latest deformation data from the sensor"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return None
        
        try:
            return self.sensor.getDeformation2D()
        except Exception as e:
            self.last_message = f"ERR: Failed to get deformation data: {e}"
            return None

    def get_latest_shear(self):
        """Get the latest shear data from the sensor"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return None
        
        try:
            return self.sensor.getShear()
        except Exception as e:
            self.last_message = f"ERR: Failed to get shear data: {e}"
            return None
    
    def get_deformation_mesh(self, scale=20.0, grid_n=15):
        """
        Capture deformation arrow mesh data with coordinates, magnitudes, and directions.
        
        Args:
            scale (float): Scale factor for arrow vectors (default: 20.0)
            grid_n (int): Number of arrows per axis (default: 15, creates 15x15 grid = 225 arrows)
        
        Returns:
            dict or None: Dictionary with 'start_coords', 'magnitudes', and 'directions', or None if unavailable
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for deformation mesh capture"
            return None
        
        if not NUMPY_AVAILABLE:
            self.last_message = "ERR: Numpy not available for deformation mesh capture"
            return None
        
        try:
            # Get current deformation data
            deformation = self.sensor.getDeformation2D()
            
            if deformation is None:
                self.last_message = "ERR: No deformation data available"
                return None
            
            # Calculate arrow mesh using the same function from main_bota.py
            start_coords, magnitudes, directions = self._calculate_arrow_mesh(
                deformation, scale=scale, grid_n=grid_n
            )
            
            if len(start_coords) == 0:
                self.last_message = "ERR: Empty deformation mesh generated"
                return None
            
            # Return mesh data as dictionary
            mesh_data = {
                'start_coords': start_coords,      # (N, 2) array of (x, y) start points
                'magnitudes': magnitudes,          # (N,) array of arrow magnitudes
                'directions': directions,          # (N, 2) array of (dx, dy) directions (unit vectors)
                'grid_size': grid_n,              # Grid dimensions for reference
                'scale_factor': scale             # Scale factor used
            }
            
            self.last_message = f"Deformation mesh captured: {len(start_coords)} arrows"
            return mesh_data
            
        except Exception as e:
            self.last_message = f"ERR: Failed to capture deformation mesh: {e}"
            return None
    
    def get_shear_mesh(self, scale=20.0, grid_n=15):
        """
        Capture shear arrow mesh data with coordinates, magnitudes, and directions.
        
        Args:
            scale (float): Scale factor for arrow vectors (default: 20.0)
            grid_n (int): Number of arrows per axis (default: 15, creates 15x15 grid = 225 arrows)
        
        Returns:
            dict or None: Dictionary with 'start_coords', 'magnitudes', and 'directions', or None if unavailable
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for shear mesh capture"
            return None
        
        if not NUMPY_AVAILABLE:
            self.last_message = "ERR: Numpy not available for shear mesh capture"
            return None
        
        try:
            # Get current shear data
            shear = self.sensor.getShear()
            
            if shear is None:
                self.last_message = "ERR: No shear data available"
                return None
            
            # Calculate arrow mesh using the same function from main_bota.py
            start_coords, magnitudes, directions = self._calculate_arrow_mesh(
                shear, scale=scale, grid_n=grid_n
            )
            
            if len(start_coords) == 0:
                self.last_message = "ERR: Empty shear mesh generated"
                return None
            
            # Return mesh data as dictionary
            mesh_data = {
                'start_coords': start_coords,      # (N, 2) array of (x, y) start points
                'magnitudes': magnitudes,          # (N,) array of arrow magnitudes  
                'directions': directions,          # (N, 2) array of (dx, dy) directions (unit vectors)
                'grid_size': grid_n,              # Grid dimensions for reference
                'scale_factor': scale             # Scale factor used
            }
            
            self.last_message = f"Shear mesh captured: {len(start_coords)} arrows"
            return mesh_data
            
        except Exception as e:
            self.last_message = f"ERR: Failed to capture shear mesh: {e}"
            return None
    
    def get_deformation_integral(self, scale=20.0, grid_n=15):
        """
        Calculate the deformation integral by summing the magnitudes of all arrows in the deformation mesh.
        This provides a single scalar value representing the overall deformation intensity.
        
        Args:
            scale (float): Scale factor for arrow vectors (default: 20.0)
            grid_n (int): Number of arrows per axis (default: 15, creates 15x15 grid = 225 arrows)
        
        Returns:
            float or None: Sum of all deformation arrow magnitudes, or None if unavailable
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for deformation integral calculation"
            return None
        
        if not NUMPY_AVAILABLE:
            self.last_message = "ERR: Numpy not available for deformation integral calculation"
            return None
        
        try:
            # Get current deformation data
            deformation = self.sensor.getDeformation2D()
            
            if deformation is None:
                self.last_message = "ERR: No deformation data available for integral calculation"
                return None
            
            # Calculate arrow mesh
            start_coords, magnitudes, directions = self._calculate_arrow_mesh(
                deformation, scale=scale, grid_n=grid_n
            )
            
            if len(magnitudes) == 0:
                self.last_message = "ERR: Empty deformation mesh for integral calculation"
                return None
            
            # Calculate the integral (sum of all magnitudes)
            deformation_integral = float(np.sum(magnitudes))
            
            self.last_message = f"Deformation integral calculated: {deformation_integral:.6f} from {len(magnitudes)} arrows"
            return deformation_integral
            
        except Exception as e:
            self.last_message = f"ERR: Failed to calculate deformation integral: {e}"
            return None
    
    def get_resultant_shear_vector(self, scale=20.0, grid_n=15):  
        """
        Calculate the resultant shear force vector by summing all shear arrow vectors in the mesh.
        This provides the x and y components of the net shear force across the sensor surface.
        
        Args:
            scale (float): Scale factor for arrow vectors (default: 20.0)
            grid_n (int): Number of arrows per axis (default: 15, creates 15x15 grid = 225 arrows)
        
        Returns:
            tuple or None: ((shear_force_x, shear_force_y), force_magnitude) components of resultant shear force vector in Newtons, or None if unavailable
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for resultant shear vector calculation"
            return None
        
        if not NUMPY_AVAILABLE:
            self.last_message = "ERR: Numpy not available for resultant shear vector calculation"
            return None
        
        try:
            # Get current shear data
            shear = self.sensor.getShear()
            
            if shear is None:
                self.last_message = "ERR: No shear data available for resultant vector calculation"
                return None
            
            # Calculate arrow mesh
            start_coords, magnitudes, directions = self._calculate_arrow_mesh(
                shear, scale=scale, grid_n=grid_n
            )
            
            if len(magnitudes) == 0 or len(directions) == 0:
                self.last_message = "ERR: Empty shear mesh for resultant vector calculation"
                return None
            
            # Calculate resultant vector by summing all vector components
            # Ensure we have numpy arrays for calculations
            magnitudes_array = np.array(magnitudes)
            directions_array = np.array(directions)
            
            # Each direction is a unit vector, so multiply by magnitude to get actual vector
            vectors = directions_array * magnitudes_array.reshape(-1, 1)  # Shape: (N, 2)
            
            # Sum all vectors to get resultant
            resultant_vector = np.sum(vectors, axis=0)
            shear_x = float(resultant_vector[0])
            shear_y = float(resultant_vector[1])
            
            # Calculate resultant magnitude 
            resultant_magnitude = float(np.linalg.norm(resultant_vector))

            # Convert magnitude to force using the provided formula: shear_force = 0.0051*magnitude - 0.4432
            force_magnitude = 0.0051 * resultant_magnitude - 0.4432
            
            # Calculate force components by scaling the unit vector by the force magnitude
            if resultant_magnitude > 0:
                force_unit_x = shear_x / resultant_magnitude
                force_unit_y = shear_y / resultant_magnitude
                shear_force_x = force_magnitude * force_unit_x
                shear_force_y = force_magnitude * force_unit_y
            else:
                shear_force_x = 0.0
                shear_force_y = 0.0
                force_magnitude = 0.0
            
            # Apply baseline correction to get net shear force components
            net_shear_force_x = shear_force_x - self.baseline_shear_x
            net_shear_force_y = shear_force_y - self.baseline_shear_y
            
            self.last_message = f"Resultant shear force calculated: ({net_shear_force_x:.6f}, {net_shear_force_y:.6f})N, magnitude: {force_magnitude:.6f}N from {len(magnitudes)} arrows"
            
            return (net_shear_force_x, net_shear_force_y), force_magnitude
            
        except Exception as e:
            self.last_message = f"ERR: Failed to calculate resultant shear vector: {e}"
            return None
    
    def _calculate_arrow_mesh(self, arrows, scale=1.0, grid_n=15):
        """
        Given a 2D vector field (H, W, 2), returns:
            - start_coords: (N, 2) array of (x, y) start points
            - magnitudes: (N,) array of arrow magnitudes
            - directions: (N, 2) array of (dx, dy) directions (unit vectors)
        The grid is sampled at grid_n x grid_n points.
        
        This is adapted from the calculate_arrow_mesh function in main_bota.py
        """
        if not NUMPY_AVAILABLE:
            return [], [], []
        
        H, W = arrows.shape[:2]
        # grid_n is the number of arrows per axis
        y_idx = np.linspace(0, H-1, grid_n, dtype=int)
        x_idx = np.linspace(0, W-1, grid_n, dtype=int)
        grid_y, grid_x = np.meshgrid(y_idx, x_idx, indexing='ij')
        
        # Get start coordinates
        start_coords = np.stack([grid_x, grid_y], axis=-1).reshape(-1, 2)
        
        # Get arrow vectors at those points
        vectors = arrows[grid_y, grid_x] * scale  # shape (grid_n, grid_n, 2)
        vectors = vectors.reshape(-1, 2)
        
        # Magnitude
        magnitudes = np.linalg.norm(vectors, axis=1)
        
        # Directions (unit vectors, avoid division by zero)
        with np.errstate(divide='ignore', invalid='ignore'):
            directions = np.where(magnitudes[:, None] > 0, vectors / magnitudes[:, None], 0)
        
        return start_coords, magnitudes, directions
    
    def capture_snapshot(self, scale=20.0, grid_n=15):
        """
        Capture a complete snapshot of all sensor data including depth array and mesh data.
        
        Args:
            scale (float): Scale factor for arrow vectors (default: 20.0)
            grid_n (int): Number of arrows per axis (default: 15)
        
        Returns:
            dict or None: Complete snapshot data, or None if sensor unavailable
        """
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for snapshot capture"
            return None
        
        try:
            # Capture all data components
            depth_array = self.sensor.getDepth()
            deformation_mesh = self.get_deformation_mesh(scale=scale, grid_n=grid_n)
            shear_mesh = self.get_shear_mesh(scale=scale, grid_n=grid_n)
            
            # Get current intensity values
            max_intensity = self.max_depth_intensity
            baseline_intensity = self.baseline_intensity
            net_intensity = self.net_intensity
            
            # Create complete snapshot
            snapshot = {
                'timestamp': time.time(),
                'depth_array': depth_array,
                'deformation_mesh': deformation_mesh,
                'shear_mesh': shear_mesh,
                'max_intensity': max_intensity,
                'baseline_intensity': baseline_intensity,
                'net_intensity': net_intensity,
                'sensor_fps': self.sensor_fps,
                'centerline_detected': self.centerline_points is not None,
                'angle_offset': self.angle_offset,
                'contact_threshold': self.contact_threshold
            }
            
            self.last_message = f"Complete snapshot captured at {time.strftime('%H:%M:%S')}"
            return snapshot
            
        except Exception as e:
            self.last_message = f"ERR: Failed to capture complete snapshot: {e}"
            return None


class BotaManager:
    """Manager for Bota force sensor integration"""
    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256
    
    def __init__(self, ifname="\\Device\\NPF_{6B61C18B-8290-4FF1-A5C0-3C01D5676AE1}"):
        self._ifname = ifname
        self._master = None
        self._expected_slave_mapping = {}
        
        # State variables
        self.connected = False
        self.running = False
        self.time_step = 1.0
        self.sampling_rate = None
        self.last_message = "Not initialized"
        
        # Force data
        self.fz_offset = 0.0
        self.current_fz = 0.0
        
        # Recording state
        self.recording = False
        self.fz_samples = []
        self.record_start_time = None
        self.last_snapshot_time = None  # Track last snapshot time
        
        # Latest sensor data
        self.latest_data = {
            'status': 0,
            'warningsErrorsFatals': 0,
            'Fx': 0.0, 'Fy': 0.0, 'Fz': 0.0,
            'Mx': 0.0, 'My': 0.0, 'Mz': 0.0,
            'forceTorqueSaturated': 0,
            'temperature': 0.0
        }
        
        # Initialize pysoem components if available
        if BOTA_AVAILABLE:
            self._master = pysoem.Master()
            SlaveSet = collections.namedtuple('SlaveSet', 'slave_name product_code config_func')
            self._expected_slave_mapping = {0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

    def bota_sensor_setup(self, slave_pos):
        """Configure the Bota sensor"""
        if not BOTA_AVAILABLE:
            return
        
        slave = self._master.slaves[slave_pos]
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))
        try:
            slave.sdo_write(0x8011, 0, struct.pack('h', 100))
        except Exception:
            pass
        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        if sampling_rate > 0:
            self.time_step = 1.0/float(sampling_rate)
        self.sampling_rate = sampling_rate

    def connect(self):
        """Initialize and configure the Bota sensor"""
        if not BOTA_AVAILABLE:
            self.last_message = "Bota sensor dependencies not available"
            return False
        
        try:
            self._master.open(self._ifname)
            if self._master.config_init() > 0:
                for i, slave in enumerate(self._master.slaves):
                    if slave.man != self.BOTA_VENDOR_ID or slave.id != self._expected_slave_mapping[i].product_code:
                        self.last_message = f"Unexpected slave configuration"
                        return False
                    slave.config_func = self._expected_slave_mapping[i].config_func
                
                self._master.config_map()
                if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                    self.last_message = "Failed to reach SAFEOP state"
                    return False
                
                self._master.state = pysoem.OP_STATE
                self._master.write_state()
                self._master.state_check(pysoem.OP_STATE, 50000)
                if self._master.state != pysoem.OP_STATE:
                    self.last_message = "Failed to reach OP state"
                    return False
                
                self.connected = True
                self.running = True
                self.last_message = "Connected and operational"
                print("✓ Bota force sensor connected")
                return True
            else:
                self.last_message = "No slaves found"
                return False
        except Exception as e:
            self.last_message = f"Connection error: {str(e)}"
            return False

    def disconnect(self):
        """Disconnect and clean up the Bota sensor"""
        self.running = False
        if self.connected and BOTA_AVAILABLE:
            try:
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()
                self.connected = False
                self.last_message = "Disconnected"
                print("✓ Bota force sensor disconnected")
            except Exception as e:
                self.last_message = f"Cleanup error: {str(e)}"

    def zero_fz(self):
        """Zero the Fz reading (calibration)"""
        if self.connected:
            self.fz_offset = self.latest_data['Fz'] + self.fz_offset  # Account for current offset
            self.last_message = f"Fz zeroed at {self.fz_offset:.4f}N"
            print(f"✓ Bota Fz zeroed at {self.fz_offset:.4f}N")
            return True
        return False

    def start_recording(self, duration_seconds=60):
        """Start Fz recording for specified duration"""
        if self.connected and not self.recording:
            self.recording = True
            self.fz_samples = []  # Will store (time, abs_fz) tuples
            self.record_start_time = time.time()
            self.last_snapshot_time = 0  # Initialize snapshot timer
            self.last_message = f"Recording started ({duration_seconds}s)"
            print(f"🔴 Started {duration_seconds}-second Fz recording...")
            return True
        return False
    
    def stop_recording(self):
        """Stop and discard current recording session"""
        if self.recording:
            self.recording = False
            sample_count = len(self.fz_samples)
            snapshot_count = self.last_snapshot_time if self.last_snapshot_time else 0
            self.fz_samples = []  # Clear collected data
            self.record_start_time = None
            self.last_snapshot_time = None  # Reset snapshot timer
            self.last_message = "Recording stopped and discarded"
            print(f"⏹️ Recording stopped and discarded ({sample_count} samples)")
            return True
        return False

    def get_recording_data(self):
        """Get current recording data (non-destructive)"""
        if self.recording or self.fz_samples:
            return {
                'fz_samples': self.fz_samples.copy(),
                'record_start_time': self.record_start_time,
                'is_recording': self.recording,
                'sample_count': len(self.fz_samples),
                'elapsed_time': time.time() - self.record_start_time if self.record_start_time else 0
            }
        return None

    def start_reader_thread(self, system_state):
        """Start the sensor reading thread"""
        if not self.connected or not BOTA_AVAILABLE:
            return None
        
        def reader_thread():
            try:
                while self.running and system_state.running:
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)
                    
                    # Parse sensor data
                    sensor_input_as_bytes = self._master.slaves[0].input
                    status = struct.unpack_from('B', sensor_input_as_bytes, 0)[0]
                    warningsErrorsFatals = struct.unpack_from('I', sensor_input_as_bytes, 1)[0]
                    Fx = struct.unpack_from('f', sensor_input_as_bytes, 5)[0]
                    Fy = struct.unpack_from('f', sensor_input_as_bytes, 9)[0]
                    Fz = struct.unpack_from('f', sensor_input_as_bytes, 13)[0]
                    Mx = struct.unpack_from('f', sensor_input_as_bytes, 17)[0]
                    My = struct.unpack_from('f', sensor_input_as_bytes, 21)[0]
                    Mz = struct.unpack_from('f', sensor_input_as_bytes, 25)[0]
                    forceTorqueSaturated = struct.unpack_from('H', sensor_input_as_bytes, 29)[0]
                    temperature = struct.unpack_from('f', sensor_input_as_bytes, 57)[0]
                    
                    # Apply Fz offset and store current value
                    self.current_fz = Fz - self.fz_offset
                    
                    # Update latest data
                    self.latest_data = {
                        'status': status,
                        'warningsErrorsFatals': warningsErrorsFatals,
                        'Fx': Fx, 'Fy': Fy, 'Fz': self.current_fz,
                        'Mx': Mx, 'My': My, 'Mz': Mz,
                        'forceTorqueSaturated': forceTorqueSaturated,
                        'temperature': temperature
                    }
                    
                    # Handle recording if active
                    if self.recording:
                        current_time = time.time()
                        elapsed_time = current_time - self.record_start_time
                        self.fz_samples.append((elapsed_time, abs(self.current_fz)))
                    
                    time.sleep(self.time_step)
                    
            except Exception as e:
                self.last_message = f"Reader error: {str(e)}"
                self.connected = False
            finally:
                if BOTA_AVAILABLE:
                    try:
                        self._master.state = pysoem.INIT_STATE
                        self._master.write_state()
                        self._master.close()
                    except:
                        pass
        
        thread = threading.Thread(target=reader_thread, name="BotaSensorThread", daemon=True)
        thread.start()
        return thread

    def get_status(self):
        """Get current sensor status"""
        return {
            'connected': self.connected,
            'recording': self.recording,
            'fz_offset': self.fz_offset,
            'current_fz': self.current_fz,
            'latest_data': self.latest_data.copy(),
            'last_message': self.last_message,
            'sampling_rate': self.sampling_rate,
            'sample_count': len(self.fz_samples) if self.fz_samples else 0,
            'elapsed_time': time.time() - self.record_start_time if self.record_start_time else 0
        }


class SensorManager:
    """Main sensor manager that coordinates all sensors"""
    
    def __init__(self):
        self.visuotactile = DaimonManager()
        self.bota = BotaManager()
        self.available = {
            'visuotactile': SENSOR_AVAILABLE,
            'bota': BOTA_AVAILABLE,
        }
    
    def initialize(self):
        """Initialize all sensor connections"""
        print("Initializing sensor connections...")
        
        # Initialize visuotactile sensor
        if self.available['visuotactile']:
            self.available['visuotactile'] = self.visuotactile.connect()
            if self.available['visuotactile']:
                # Calibrate baseline intensity
                print("Calibrating baseline intensity...")
                if self.visuotactile.calibrate_baseline_intensity():
                    print("✓ Baseline intensity calibrated")
                else:
                    print("⚠ Baseline calibration failed")
                
                # Calibrate baseline shear force components
                print("Calibrating baseline shear force...")
                if self.visuotactile.calibrate_baseline_shear_force():
                    print("✓ Baseline shear force calibrated")
                else:
                    print("⚠ Baseline shear force calibration failed")
        
        # Initialize Bota force sensor
        if self.available['bota']:
            self.available['bota'] = self.bota.connect()
            if self.available['bota']:
                # Zero the Bota force sensor readings
                print("Zeroing Bota force sensor...")
                if self.bota.zero_fz():
                    print("✓ Bota force sensor zeroed")
                else:
                    print("⚠ Bota force sensor zeroing failed")
        
        return self.available
    
    def cleanup(self):
        """Clean up sensor connections"""
        print("Cleaning up sensor connections...")
        self.visuotactile.disconnect()
        self.bota.disconnect()
    
    def get_status(self):
        """Get status of all sensors"""
        return {
            'visuotactile': {
                'connected': self.visuotactile.connected,
                'last_message': self.visuotactile.last_message,
                'max_intensity': self.visuotactile.max_depth_intensity,
                'baseline': self.visuotactile.baseline_intensity,
                'net_intensity': self.visuotactile.net_intensity,
                'sensor_fps': self.visuotactile.sensor_fps,
                'centerline_detected': self.visuotactile.centerline_points is not None,
                'angle_offset': self.visuotactile.angle_offset,
                'contact_threshold': self.visuotactile.contact_threshold
            },
            'bota': self.bota.get_status()
        }
    
    def check_gripping_condition(self, net_intensity):
        """Check if the net intensity exceeds the threshold during gripping"""
        return net_intensity > ADAPTIVE_GRIPPING_CONFIG["threshold"]
