"""
Main application entry point for the Gripper Control System
Handles UI and the main loop, coordinates all modules
"""

import time
import threading
import signal
import sys
import os
import cv2

from pynput import keyboard

# Import for finding hardware devices on Windows
try:
    import wmi
except ImportError:
    print("Warning: The 'wmi' library is required for camera detection.")
    print("Camera features will be disabled. Install with: pip install wmi")
    wmi = None

from src.core.config_loader import CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, RECORDING_CONFIG, SAFETY_CONFIG
from src.core.hardware_manager import HardwareManager
from src.core.sensor_manager import SensorManager
from src.core.data_manager import DataManager


class SystemState:
    """A class to hold the live state of the entire system"""
    def __init__(self):
        # Control mode state - always step-based
        self.object_diameter_mm = CONFIG["initial_diameter_mm"]
        
        # System state
        self.running = True
        
        # Hardware managers
        self.hardware = HardwareManager()
        self.sensors = SensorManager()
        
        # Data recording manager
        self.data_manager = DataManager()
        
        # Thread references
        self.threads = []
    
    def update_diameter(self, diameter_mm):
        """Update diameter (control mode is always step-based)"""
        self.object_diameter_mm = diameter_mm


class GripperControlSystem:
    """Main control system class"""
    
    def __init__(self):
        self.state = SystemState()
        self.listener = None
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle interrupt signals for graceful shutdown"""
        print(f"\n\nReceived signal {signum}. Shutting down gracefully...")
        self.state.running = False
    
    def initialize(self):
        """Initialize all system components"""
        print("Initializing Complete Gripper Control System...")
        
        # Initialize hardware
        hardware_status = self.state.hardware.initialize()
        
        # Initialize sensors
        sensor_status = self.state.sensors.initialize()
        
        # Register safety callback for emergency gripper opening
        if self.state.sensors.visuotactile.connected and self.state.hardware.gripper.connected:
            def emergency_gripper_open():
                """Emergency callback to open gripper when safety threshold is exceeded"""
                try:
                    self.state.hardware.gripper.move_to_percentage(
                        SAFETY_CONFIG["emergency_open_percent"], 
                        velocity=ADAPTIVE_GRIPPING_CONFIG["opening_velocity"]
                    )
                except Exception as e:
                    print(f"❌ Emergency gripper opening failed: {e}")
            
            self.state.sensors.visuotactile.set_safety_callback(emergency_gripper_open)
            print(f"✓ Safety monitoring enabled: X-threshold = {SAFETY_CONFIG['shear_x_threshold_n']}N, Y-threshold = {SAFETY_CONFIG['shear_y_threshold_n']}N")
        
        # Start background threads
        self._start_background_threads()
        
        # Initialize gripper to default open position
        if self.state.hardware.gripper.connected:
            print(f"Initializing gripper to default open position ({CONFIG['gripper_default_open_percent']}% closed)...")
            self.state.hardware.gripper.move_to_percentage(CONFIG["gripper_default_open_percent"])
            time.sleep(2)
        
        # Perform UR robot ready signal (move arm up 3cm to indicate system is ready)
        if self.state.hardware.ur_robot.connected:
            print("Signaling system ready with UR robot arm movement...")
            self.state.hardware.ur_robot.perform_ready_signal()
        
        print("\n" + "="*60)
        print("           SYSTEM READY - STARTING CONTROL LOOP")
        print("="*60)
        print()
        
        return hardware_status and sensor_status
    
    def _start_background_threads(self):
        """Start all background threads"""
        # Start stepper reader thread
        if self.state.hardware.stepper.connected:
            stepper_thread = self.state.hardware.stepper.start_reader_thread(self.state)
            if stepper_thread:
                self.state.threads.append(stepper_thread)
                print("✓ Stepper reader thread started")
        
        # Start sensor reader thread
        if self.state.sensors.visuotactile.connected:
            sensor_thread = self.state.sensors.visuotactile.start_reader_thread(self.state)
            if sensor_thread:
                self.state.threads.append(sensor_thread)
                print("✓ Sensor reader thread started")
        
        # Start Bota force sensor reader thread
        if self.state.sensors.bota.connected:
            bota_thread = self.state.sensors.bota.start_reader_thread(self.state)
            if bota_thread:
                self.state.threads.append(bota_thread)
                print("✓ Bota force sensor reader thread started")
        
        # Give sensor reader time to start and get initial readings
        if self.state.sensors.visuotactile.connected:
            print("Waiting for sensor to stabilize...")
            time.sleep(1.0)
    
    def cleanup(self):
        """Clean up all system components"""
        print("\nExiting program...")
        self.state.running = False
        
        # Stop the keyboard listener first
        if self.listener:
            try:
                self.listener.stop()
                self.listener.join(timeout=2.0)
                print("✓ Keyboard listener stopped")
            except:
                pass
        
        # Clean up hardware and sensors
        self.state.hardware.cleanup()
        self.state.sensors.cleanup()
        
        # Clean up data manager
        if self.state.data_manager.is_recording():
            print("Stopping active recording session...")
            self.state.data_manager.stop_recording_session()
        
        # Force stop all background threads
        print("Stopping background threads...")
        active_threads = threading.enumerate()
        main_thread = threading.main_thread()
        
        for thread in active_threads:
            if thread != main_thread and thread.is_alive():
                try:
                    print(f"Waiting for thread: {thread.name}")
                    thread.join(timeout=2.0)
                    if thread.is_alive():
                        print(f"⚠ Thread {thread.name} did not stop gracefully")
                except Exception as e:
                    print(f"⚠ Error stopping thread {thread.name}: {e}")
        
        # Close any remaining matplotlib windows
        try:
            import matplotlib.pyplot as plt
            plt.close('all')
            print("✓ Matplotlib windows closed")
        except:
            pass
        
        # Close any remaining OpenCV windows
        try:
            import cv2
            cv2.destroyAllWindows()
            print("✓ OpenCV windows closed")
        except:
            pass
        
        print("✓ Cleanup completed")
        print("Exiting program...")
        
        # Force exit after cleanup
        os._exit(0)
    
    def update_display(self):
        """Update the system display"""
        # Clear screen using ANSI escape codes
        print('\033[2J\033[H', end='')
        sys.stdout.flush()
        
        print("=" * 60)
        print("         STEP-BASED GRIPPER CONTROL SYSTEM")
        print("=" * 60)
        
        # Get hardware status
        hw_status = self.state.hardware.get_status()
        sensor_status = self.state.sensors.get_status()
        
        # Stepper motor status
        print(f"ROTATION CONTROL (Step-Based):")
        print(f"  Motor Position: M1={hw_status['stepper']['m1_pos']:<4} | M2={hw_status['stepper']['m2_pos']:<4}")
        print(f"  Target Steps:   {CONFIG['fixed_target_steps']:>6}")
        stepper_status = "Connected" if hw_status['stepper']['connected'] else "Disconnected"
        print(f"  Status: {stepper_status} | {hw_status['stepper']['last_message']}")
        
        print("-" * 60)
        
        # Gripper status
        gripper_status = "Connected" if hw_status['gripper']['connected'] else "Disconnected"
        gripper_closure = hw_status['gripper']['closure_percent']
        
        print("GRIPPER CONTROL (DM Motor):")
        print(f"  Closure: {gripper_closure:.1f}% | Position: {hw_status['gripper']['position_rad']:.3f} rad")
        print(f"  Status: {gripper_status} | {hw_status['gripper']['last_message']}")
        
        print("-" * 60)
        
        # Sensor status
        sensor_available = "Connected" if sensor_status['visuotactile']['connected'] else "Disconnected"
        print("DEPTH SENSOR:")
        print(f"  Max Intensity: {sensor_status['visuotactile']['max_intensity']:.3f}")
        print(f"  Baseline: {sensor_status['visuotactile']['baseline']:.3f} | Net: {sensor_status['visuotactile']['net_intensity']:.3f}")
        print(f"  Status: {sensor_available}")
        
        print("-" * 60)
        
        # Bota Force Sensor status
        bota_status = sensor_status['bota']
        bota_connected = "Connected" if bota_status['connected'] else "Disconnected"
        print("BOTA FORCE SENSOR:")
        print(f"  Force Z (Fz): {bota_status['current_fz']:.4f} N | Offset: {bota_status['fz_offset']:.4f} N")
        print(f"  Status: {bota_connected} | {bota_status['last_message']}")
        
        print("-" * 60)
        
        # Safety monitoring status
        if sensor_status['visuotactile']['connected']:
            safety_status = self.state.sensors.visuotactile.get_safety_status()
            safety_enabled = "🟢 Enabled" if safety_status['safety_enabled'] else "🔴 Disabled"
            safety_state = "🚨 TRIGGERED" if safety_status['safety_triggered'] else "✅ Normal"
            print("SAFETY MONITORING:")
            print(f"  Status: {safety_enabled} | State: {safety_state}")
            print(f"  X-Threshold: {safety_status['shear_x_threshold']:.1f}N | Y-Threshold: {safety_status['shear_y_threshold']:.1f}N")
            print(f"  Current X: {safety_status['last_shear_x']:.3f}N | Current Y: {safety_status['last_shear_y']:.3f}N")
            if safety_status['safety_triggered'] and safety_status['safety_trigger_reason']:
                print(f"  Trigger Reason: {safety_status['safety_trigger_reason']}")
            if safety_status['time_since_last_trigger'] < 10:  # Show recent triggers
                print(f"  Last Trigger: {safety_status['time_since_last_trigger']:.1f}s ago")
        else:
            print("SAFETY MONITORING:")
            print(f"  Status: ⚪ Unavailable (sensor disconnected)")
        
        print("-" * 60)
        
        # UR Robot status
        ur_status = "Connected" if hw_status['ur_robot']['connected'] else "Disconnected"
        print("UR ROBOT CONTROL:")
        if hw_status['ur_robot']['current_pose']:
            pose = hw_status['ur_robot']['current_pose']
            print(f"  Position: X={pose[0]:.3f} | Y={pose[1]:.3f} | Z={pose[2]:.3f}")
            print(f"  Rotation: RX={pose[3]:.3f} | RY={pose[4]:.3f} | RZ={pose[5]:.3f}")
        print(f"  Status: {ur_status} | {hw_status['ur_robot']['last_message']}")
        
        print("-" * 60)
        
        # Data Recording status
        recording_info = self.state.data_manager.get_session_info()
        recording_status = "🔴 Recording" if recording_info['status'] == 'recording' else "⚪ Idle"
        print("DATA RECORDING:")
        print(f"  Status: {recording_status}")
        if recording_info['status'] == 'recording':
            print(f"  Session: {recording_info['session_name']} | Elapsed: {recording_info['elapsed_time_s']:.1f}s")
        
        print("-" * 60)
        
        # Controls
        print("CONTROLS:")
        print("  UR Robot:  [Q/E] Up/Down | [A/D] Left/Right | [W/S] Forward/Backward")
        print("  Gripper Rotation: [J] CCW | [L] CW")
        print("  Gripper:   [O] Open (50%) | [C] Close (Adaptive) | [I/K] +/-1% Manual")
        print("  Sensor:    [R] Calibrate Baseline | [Z] Zero Force Sensor | [B] Zero Shear Force | [T] Toggle Safety")
        print("  Recording: [F] Start Session | [G] Stop Session")
        print("  General:   [H] Home Motors* | [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
        print("             *Homing only works when gripper is at default open position (≤50% closed)")
        print("=" * 60)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            # --- UR Robot Controls ---
            if key.char == 'q':  # UR Robot Up
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('up')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'e':  # UR Robot Down
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('down')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'a':  # UR Robot Left
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('left')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'd':  # UR Robot Right
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('right')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'w':  # UR Robot Forward
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('forward')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 's':  # UR Robot Backward
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('backward')
                else:
                    print("❌ UR Robot not connected")

            # --- Stepper Motor Controls (Gripper Rotation) ---
            elif key.char == 'j':  # Rotate CCW
                # Set fixed motor speed
                self.state.hardware.stepper.speed = CONFIG["fixed_motor_speed"]
                
                print(f"🚀 Starting CCW step move: Target steps = {CONFIG['fixed_target_steps']}")
                
                # Step-based control for all diameters
                self.state.hardware.stepper.send_step_move_command(
                    CONFIG["fixed_target_steps"], 'ccw'
                )
                    
            elif key.char == 'l':  # Rotate CW
                # Set fixed motor speed
                self.state.hardware.stepper.speed = CONFIG["fixed_motor_speed"]
                
                print(f"🚀 Starting CW step move: Target steps = {CONFIG['fixed_target_steps']}")
                
                # Step-based control for all diameters
                self.state.hardware.stepper.send_step_move_command(
                    CONFIG["fixed_target_steps"], 'cw'
                )
            
            # --- Object Diameter Adjustment ---
            elif key.char == '1':
                self.state.update_diameter(1.0)
            elif key.char == '2':
                self.state.update_diameter(2)
            elif key.char == '3':
                self.state.update_diameter(3)
            elif key.char == '4':
                self.state.update_diameter(4)
            elif key.char == '5':
                self.state.update_diameter(5)

            # --- Gripper Controls ---
            elif key.char == 'o':  # Open gripper to default position
                # Use opening velocity for faster gripper opening
                self.state.hardware.gripper.move_to_percentage(CONFIG["gripper_default_open_percent"], 
                                                             ADAPTIVE_GRIPPING_CONFIG["opening_velocity"])
            elif key.char == 'c':  # Close gripper (adaptive)
                if not self.state.hardware.gripper.is_gripping:
                    self.state.hardware.gripper.move_to_percentage(95)
                else:
                    self.state.hardware.gripper.last_message = "Gripping already in progress"
            elif key.char == 'i':  # Increase gripper close percentage by 1%
                if self.state.hardware.gripper.connected:
                    current_closure = self.state.hardware.gripper.gripper_closure_percent
                    new_closure = min(100.0, current_closure + 1.0)  # Cap at 100%
                    self.state.hardware.gripper.move_to_percentage(new_closure)
                    print(f"🔧 Gripper closure: {current_closure:.1f}% → {new_closure:.1f}%")
                else:
                    print("❌ Gripper not connected")
            elif key.char == 'k':  # Decrease gripper close percentage by 1%
                if self.state.hardware.gripper.connected:
                    current_closure = self.state.hardware.gripper.gripper_closure_percent
                    new_closure = max(0.0, current_closure - 1.0)  # Floor at 0%
                    self.state.hardware.gripper.move_to_percentage(new_closure)
                    print(f"🔧 Gripper closure: {current_closure:.1f}% → {new_closure:.1f}%")
                else:
                    print("❌ Gripper not connected")

            # --- Sensor Controls ---
            elif key.char == 'r':  # Calibrate baseline intensity
                if self.state.sensors.visuotactile.sensor:
                    self.state.sensors.visuotactile.calibrate_baseline_intensity()
                else:
                    self.state.sensors.visuotactile.last_message = "ERR: Sensor not connected"
            elif key.char == 'z':  # Zero Bota force sensor readings
                if self.state.sensors.bota.connected:
                    if self.state.sensors.bota.zero_fz():
                        print("✅ Bota force sensor readings zeroed")
                    else:
                        print("❌ Failed to zero Bota force sensor")
                else:
                    print("❌ Bota force sensor not connected")
            elif key.char == 'b':  # Zero baseline shear force components
                if self.state.sensors.visuotactile.connected:
                    if self.state.sensors.visuotactile.calibrate_baseline_shear_force():
                        print("✅ Baseline shear force components zeroed")
                    else:
                        print("❌ Failed to zero baseline shear force")
                else:
                    print("❌ Depth sensor not connected")
            elif key.char == 't':  # Toggle safety monitoring
                if self.state.sensors.visuotactile.connected:
                    # Toggle safety monitoring in config
                    SAFETY_CONFIG["safety_check_enabled"] = not SAFETY_CONFIG["safety_check_enabled"]
                    status = "enabled" if SAFETY_CONFIG["safety_check_enabled"] else "disabled"
                    print(f"🔧 Safety monitoring {status}")
                    
                    # Reset safety state when re-enabling
                    if SAFETY_CONFIG["safety_check_enabled"]:
                        self.state.sensors.visuotactile.safety_triggered = False
                else:
                    print("❌ Safety monitoring unavailable (sensor disconnected)")

            # --- Data Recording Controls ---
            elif key.char == 'f':  # Start recording session
                # Generate session name based on config
                base_name = RECORDING_CONFIG["default_session_name"]
                if RECORDING_CONFIG["use_timestamp_session_name"]:
                    session_name = f"{base_name}_{int(time.time())}"
                else:
                    session_name = base_name
                
                if self.state.data_manager.start_recording_session(session_name, self.state):
                    print(f"🔴 Started recording session: {session_name}")
                else:
                    print("❌ Failed to start recording session")
            elif key.char == 'g':  # Stop recording session
                if self.state.data_manager.stop_recording_session():
                    print("⏹️ Recording session stopped")
                else:
                    print("⚠️ No recording session to stop")

            # --- Utility Commands ---
            elif key.char == 'h':  # Home stepper motors (safety constraint: gripper must be at default open position or more open)
                gripper_closure = self.state.hardware.gripper.gripper_closure_percent
                if gripper_closure <= CONFIG["gripper_default_open_percent"]:  # Allow homing when at default open position or more open
                    if self.state.hardware.stepper.connected:
                        print(f"🏠 Starting Python homing sequence...")
                        print(f"⚠ Please wait ~6 seconds for homing to complete...")
                        
                        # Perform Python-based homing sequence
                        homing_speed = CONFIG["homing_speed"]
                        
                        # Step 1: Move both motors to maximum position (1000)
                        print(f"   Step 1/3: Moving to max position (1000, 1000)")
                        self.state.hardware.stepper.send_move_command(1000, 1000, homing_speed)
                        time.sleep(1.5)  # Increased delay to ensure movement completes
                        
                        # Step 2: Move both motors to minimum position (0)
                        print(f"   Step 2/3: Moving to min position (0, 0)")
                        self.state.hardware.stepper.send_move_command(0, 0, homing_speed)
                        time.sleep(2.5)  # Increased delay to ensure movement completes
                        
                        # Step 3: Move both motors to center position (500, 500)
                        print(f"   Step 3/3: Moving to center position (500, 500)")
                        self.state.hardware.stepper.send_move_command(500, 500, homing_speed)
                        time.sleep(1.0)  # Allow time for final positioning
                        
                        # Reset angle tracking to zero
                        self.state.hardware.stepper.current_angle_deg = 0.0
                        
                        self.state.hardware.stepper.last_message = "Python homing sequence completed"
                        print(f"✅ Homing sequence completed - motors at center, angle reset to 0°")
                    else:
                        print(f"❌ Cannot home: Stepper motors not connected")
                else:
                    print(f"⚠ Motor homing BLOCKED: Gripper must be at default open position (≤{CONFIG['gripper_default_open_percent']}% closed, currently {gripper_closure:.1f}% closed)")
                    print(f"📋 Press [O] to open gripper first, then retry homing with [H]")
                    self.state.hardware.stepper.last_message = f"Homing blocked: gripper {gripper_closure:.1f}% closed"
            elif key.char == 'x':  # Reset stepper position to center and angle
                self.state.hardware.stepper.reset_to_center()

        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.space:
                # Stop all motors
                self.state.hardware.stepper.stop_motors()
                self.state.hardware.stepper.last_message = "STOP command sent"
                self.state.hardware.gripper.last_message = "Manual stop requested"
            elif key == keyboard.Key.esc:
                # Stop the listener and the program
                self.state.running = False
                return False
    
    def run(self):
        """Main application loop"""
        try:
            # Initialize system
            if not self.initialize():
                print("❌ System initialization failed")
                return
            
            # Small delay to ensure clean transition to control loop
            time.sleep(0.5)

            # Start the keyboard listener
            self.listener = keyboard.Listener(on_press=self.on_press)
            self.listener.start()
            
            last_update = 0
            while self.state.running:
                try:
                    current_time = time.time()
                    
                    # Check for adaptive gripping conditions
                    if (self.state.hardware.gripper.is_gripping and 
                        self.state.sensors.visuotactile.connected):
                        net_intensity = self.state.sensors.visuotactile.net_intensity
                        if self.state.sensors.check_gripping_condition(net_intensity):
                            self.state.hardware.gripper.handle_object_detection()
                    
                    # Update display periodically
                    if current_time - last_update >= DISPLAY_CONFIG["update_interval_seconds"]:
                        self.update_display()
                        last_update = current_time
                    
                    time.sleep(0.1)  # Faster polling for keyboard input
                    
                except KeyboardInterrupt:
                    print("\n\nKeyboard interrupt received. Shutting down...")
                    break
        
        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received. Shutting down...")
        except Exception as e:
            print(f"\nAn unexpected error occurred: {e}")
        finally:
            self.cleanup()


def main():
    """Main entry point"""
    control_system = GripperControlSystem()
    control_system.run()


if __name__ == "__main__":
    main()
