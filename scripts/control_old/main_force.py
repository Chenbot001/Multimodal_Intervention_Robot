import time
import cv2
import numpy as np
import threading
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'sensors', 'daimon'))
from dmrobotics import Sensor, put_arrows_on_image

# --- Bota sensor imports and dashboard logic ---
import struct
import pysoem
import ctypes
import collections
import json


def calculate_arrow_mesh(arrows, scale=1.0, grid_n=15):
    """
    Given a 2D vector field (H, W, 2), returns:
        - start_coords: (N, 2) array of (x, y) start points
        - magnitudes: (N,) array of arrow magnitudes
        - directions: (N, 2) array of (dx, dy) directions (unit vectors)
    The grid is sampled at grid_n x grid_n points.
    """
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


def save_force_data_to_json(timestamp, mean_fz, peak_intensity, deformation_mesh, shear_mesh):
    """
    Save force data to a JSON file with timestamp-based filename.
    
    Args:
        timestamp: Recording start timestamp
        mean_fz: Mean Fz value to 4 decimal places
        peak_intensity: Peak intensity to 3 decimal places
        deformation_mesh: List of dicts with coordinates, magnitude, and direction
        shear_mesh: List of dicts with coordinates, magnitude, and direction
    """
    # Create data structure
    force_data = {
        "recording_timestamp": timestamp,
        "mean_fz": round(mean_fz, 4),
        "peak_intensity": round(peak_intensity, 3),
        "deformation_mesh": deformation_mesh,
        "shear_mesh": shear_mesh
    }
    
    # Create data directory path
    data_dir = os.path.join(os.path.dirname(__file__), "data")
    
    # Create data directory if it doesn't exist
    try:
        os.makedirs(data_dir, exist_ok=True)
    except Exception as e:
        print(f"❌ Error creating data directory: {e}")
        return
    
    # Create filename with timestamp
    timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(timestamp))
    filename = f"grip_pull_{timestamp_str}.json"
    
    # Create full file path
    file_path = os.path.join(data_dir, filename)
    
    # Save to JSON file
    try:
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(force_data, f, indent=2, ensure_ascii=False)
        print(f"✅ Force data saved to: {file_path}")
    except Exception as e:
        print(f"❌ Error saving force data: {e}")


class BotaSensorDashboard:
    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256
    time_step = 1.0

    def __init__(self, ifname):
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple(
            'SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}
        self.running = True
        self.fz_offset = 0.0
        self.last_mean_fz = None
        self.last_mean_intensity = None
        self.recording_fz = False
        self.recording_intensity = False
        self.fz_samples = []
        self.intensity_samples = []
        self.record_start_time = None

    def clear_screen(self):
        os.system('cls' if os.name == 'nt' else 'clear')

    def display_dashboard(self, data, daimon_status=None, mesh_data_status=None):
        self.clear_screen()
        print("=" * 80)
        print("                         SENSOR DASHBOARD")
        print("=" * 80)
        # Daimon sensor block
        print("DAIMON SENSOR")
        print("-" * 80)
        if daimon_status:
            print(f"Connection: {daimon_status.get('connection', 'N/A')}")
            print(f"Measurement FPS: {daimon_status.get('fps', 'N/A')}")
            print(f"Peak Intensity: {daimon_status.get('avg_top10', 'N/A')}")
        else:
            print("Connection: N/A")
            print("Measurement FPS: N/A")
            print("Avg Top-10 Depth Intensity: N/A")
        print("=" * 80)
        # Bota sensor block
        print("BOTA SENSOR")
        print("-" * 80)
        print(f"Status: {data['status']:<10} | Warnings/Errors/Fatals: {data['warningsErrorsFatals']}")
        print(f"Temperature: {data['temperature']:.2f}°C")
        print("-" * 80)
        print("FORCE & TORQUE DATA")
        print("-" * 80)
        print(f"Fx: {data['Fx']:>10.3f} N    | Fy: {data['Fy']:>10.3f} N    | Fz: {data['Fz']:>10.3f} N")
        print(f"Mx: {data['Mx']:>10.3f} Nm   | My: {data['My']:>10.3f} Nm   | Mz: {data['Mz']:>10.3f} Nm")
        print(f"Force-Torque Saturated: {data['forceTorqueSaturated']}")
        print("-" * 80)
        print("RESULTS")
        print("-" * 80)
        print(f"Fz Offset (Calibration): {self.fz_offset:.4f} N")
        if self.last_mean_fz is not None:
            print(f"Mean Fz (1s recording): {self.last_mean_fz:.4f} N")
        else:
            print("Mean Fz (1s recording): N/A")
        if self.last_mean_intensity is not None:
            print(f"Peak Intensity (1s recording): {self.last_mean_intensity:.4f}")
        else:
            print("Peak Intensity (1s recording): N/A")

        # --- MESH DATA BLOCK ---
        print("-" * 80)
        print("MESH DATA (LAST FRAME)")
        print("-" * 80)
        if mesh_data_status:
            print("DEFORMATION:")
            print(f"  - Max Magnitude: {mesh_data_status.get('def_max_mag', 'N/A')}")
            print(f"  - Coordinates:   {mesh_data_status.get('def_max_coords', 'N/A')}")
            print(f"  - Direction:     {mesh_data_status.get('def_max_dir', 'N/A')}")
            print("SHEAR:")
            print(f"  - Max Magnitude: {mesh_data_status.get('shear_max_mag', 'N/A')}")
            print(f"  - Coordinates:   {mesh_data_status.get('shear_max_coords', 'N/A')}")
            print(f"  - Direction:     {mesh_data_status.get('shear_max_dir', 'N/A')}")
        else:
            print("No mesh data processed yet.")

        print("=" * 80)
        update_rate = getattr(self, 'sampling_rate', None)
        if update_rate is None:
            try:
                update_rate = 1.0/self.time_step
            except Exception:
                update_rate = 'N/A'
        print(f"Update Rate: {update_rate} Hz | Press Ctrl+C or 'q' to stop")
        print("=" * 80)

    def bota_sensor_setup(self, slave_pos):
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

    def run(self, daimon_status_provider=None, key_provider=None, mesh_data_provider=None):
        self._master.open(self._ifname)
        if self._master.config_init() > 0:
            for i, slave in enumerate(self._master.slaves):
                assert(slave.man == self.BOTA_VENDOR_ID)
                assert(slave.id == self._expected_slave_mapping[i].product_code)
                slave.config_func = self._expected_slave_mapping[i].config_func
            self._master.config_map()
            if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.SAFEOP_STATE:
                        print(f'{slave.name} did not reach SAFEOP state')
                raise Exception('not all slaves reached SAFEOP state')
            self._master.state = pysoem.OP_STATE
            self._master.write_state()
            self._master.state_check(pysoem.OP_STATE, 50000)
            if self._master.state != pysoem.OP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.OP_STATE:
                        print(f'{slave.name} did not reach OP state')
                raise Exception('not all slaves reached OP state')
            try:
                while self.running:
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)
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
                    # Keyboard controls
                    if key_provider:
                        key = key_provider()
                        if key == 'z':
                            self.fz_offset = Fz
                        elif key == 'space' and not self.recording_fz:
                            self.recording_fz = True
                            self.recording_intensity = True
                            self.fz_samples = []
                            self.intensity_samples = []
                            self.record_start_time = time.time()
                    # Apply Fz offset
                    Fz_disp = Fz - self.fz_offset
                    # If recording, collect Fz_disp samples for 1s
                    if self.recording_fz:
                        self.fz_samples.append(Fz_disp)
                        if (time.time() - self.record_start_time) >= 1.0:
                            mean_fz = sum(self.fz_samples) / len(self.fz_samples) if self.fz_samples else 0.0
                            self.last_mean_fz = mean_fz
                            self.recording_fz = False
                    # If recording intensity, collect samples for 1s
                    if self.recording_intensity:
                        daimon_status = daimon_status_provider() if daimon_status_provider else None
                        if daimon_status and 'avg_top10' in daimon_status and isinstance(daimon_status['avg_top10'], (int, float)):
                            self.intensity_samples.append(daimon_status['avg_top10'])
                        if (time.time() - self.record_start_time) >= 1.0:
                            mean_intensity = sum(self.intensity_samples) / len(self.intensity_samples) if self.intensity_samples else 0.0
                            self.last_mean_intensity = mean_intensity
                            self.recording_intensity = False
                    data = {
                        'status': status,
                        'warningsErrorsFatals': warningsErrorsFatals,
                        'Fx': Fx, 'Fy': Fy, 'Fz': Fz_disp,
                        'Mx': Mx, 'My': My, 'Mz': Mz,
                        'forceTorqueSaturated': forceTorqueSaturated,
                        'temperature': temperature
                    }
                    daimon_status = daimon_status_provider() if daimon_status_provider else None
                    mesh_data = mesh_data_provider() if mesh_data_provider else None
                    self.display_dashboard(
                        data, 
                        daimon_status=daimon_status, 
                        mesh_data_status=mesh_data
                    )
                    time.sleep(self.time_step)
            except Exception as e:
                print(e)
            finally:
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()


if __name__ == "__main__":
    import msvcrt
    import threading # Make sure this import is here

    # Shared Daimon sensor status
    daimon_status = {
        'connection': 'Disconnected',
        'fps': 'N/A',
        'max_depth': 'N/A',
        'avg_top10': 0.0
    }
    
    # Shared dictionary for mesh data
    mesh_data_status = {
        'def_max_coords': 'N/A',
        'def_max_mag': 'N/A',
        'def_max_dir': 'N/A',
        'shear_max_coords': 'N/A',
        'shear_max_mag': 'N/A',
        'shear_max_dir': 'N/A',
    }

    def get_daimon_status():
        return daimon_status.copy()

    def get_mesh_data_status():
        return mesh_data_status.copy()

    # Keyboard provider for Bota thread
    key_state = {'last': None}
    def key_provider():
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in [b'z', b'Z']:
                return 'z'
            elif key == b' ':
                return 'space'
        return None

    # Start Bota sensor dashboard in a background thread
    bota_adapter = "\\Device\\NPF_{6B61C18B-8290-4FF1-A5C0-3C01D5676AE1}"
    bota_dashboard = BotaSensorDashboard(bota_adapter)
    bota_thread = threading.Thread(
        target=bota_dashboard.run, 
        args=(get_daimon_status, key_provider, get_mesh_data_status), 
        daemon=True
    )
    bota_thread.start()

    dev_serial_id = 0 
    sensor = Sensor(dev_serial_id)  # serial ID
    daimon_status['connection'] = 'Connected'
    frame_num = 0.0
    start_time = time.time()
    black_img = np.zeros_like(sensor.getRawImage()) 
    black_img = np.stack([black_img]*3, axis=-1) 

    # Data structures to hold recorded frames
    recorded_deformation_frames = []
    recorded_shear_frames = []
    
    # State variable to detect when a recording session ends
    was_recording = False

    try:
        while True:
            img = sensor.getRawImage()
            frame_num += 1.0
            deformation = sensor.getDeformation2D()
            shear = sensor.getShear()

            depth = sensor.getDepth() # output the deformed depth
            depth_img = cv2.applyColorMap((depth*0.25* 255.0).astype('uint8'), cv2.COLORMAP_HOT)

            # Update Daimon status
            flat = depth.flatten()
            if flat.size >= 10:
                top10 = np.partition(flat, -10)[-10:]
                avg_top10 = float(np.mean(top10))
            else:
                avg_top10 = float(np.mean(flat))
            daimon_status['avg_top10'] = round(avg_top10, 3)

            cv2.imshow('depth', depth_img)
            cv2.imshow('deformation', put_arrows_on_image(black_img, deformation*20))
            cv2.imshow('shear', put_arrows_on_image(black_img, shear*20))

            k = cv2.waitKey(3)
            if k != -1 and k != 255:  # Debug: show all key presses
                char_code = k & 0xFF
                if 32 <= char_code <= 126:  # Printable ASCII
                    print(f"[DEBUG] Key detected: {k} ('{chr(char_code)}')")
                else:
                    print(f"[DEBUG] Key detected: {k} (non-printable)")
            
            if k & 0xFF == ord('q'):
                break
            elif k & 0xFF == ord('r'):
                sensor.reset()
                print("Sensor reset")

            # Logic to capture frames during recording and process them after
            is_recording_now = bota_dashboard.recording_fz

            if is_recording_now:
                recorded_deformation_frames.append(deformation)
                recorded_shear_frames.append(shear)

            if was_recording and not is_recording_now:
                print("\n--- Recording Finished ---")
                
                if recorded_deformation_frames and recorded_shear_frames:
                    last_deformation_frame = recorded_deformation_frames[-1]
                    last_shear_frame = recorded_shear_frames[-1]

                    def_coords, def_mags, def_dirs = calculate_arrow_mesh(
                        last_deformation_frame, scale=20.0
                    )
                    max_def_idx = np.argmax(def_mags)
                    mesh_data_status['def_max_coords'] = f"({def_coords[max_def_idx][0]}, {def_coords[max_def_idx][1]})"
                    mesh_data_status['def_max_mag'] = f"{def_mags[max_def_idx]:.3f}"
                    mesh_data_status['def_max_dir'] = f"({def_dirs[max_def_idx][0]:.2f}, {def_dirs[max_def_idx][1]:.2f})"

                    shear_coords, shear_mags, shear_dirs = calculate_arrow_mesh(
                        last_shear_frame, scale=20.0
                    )
                    max_shear_idx = np.argmax(shear_mags)
                    mesh_data_status['shear_max_coords'] = f"({shear_coords[max_shear_idx][0]}, {shear_coords[max_shear_idx][1]})"
                    mesh_data_status['shear_max_mag'] = f"{shear_mags[max_shear_idx]:.3f}"
                    mesh_data_status['shear_max_dir'] = f"({shear_dirs[max_shear_idx][0]:.2f}, {shear_dirs[max_shear_idx][1]:.2f})"

                    # Prepare mesh data for JSON saving
                    deformation_mesh = []
                    for i in range(len(def_coords)):
                        deformation_mesh.append({
                            "coordinates": [int(def_coords[i][0]), int(def_coords[i][1])],
                            "magnitude": float(def_mags[i]),
                            "direction": [float(def_dirs[i][0]), float(def_dirs[i][1])]
                        })
                    
                    shear_mesh = []
                    for i in range(len(shear_coords)):
                        shear_mesh.append({
                            "coordinates": [int(shear_coords[i][0]), int(shear_coords[i][1])],
                            "magnitude": float(shear_mags[i]),
                            "direction": [float(shear_dirs[i][0]), float(shear_dirs[i][1])]
                        })
                    
                    # Get recording timestamp and sensor data for saving
                    recording_timestamp = bota_dashboard.record_start_time if hasattr(bota_dashboard, 'record_start_time') else time.time()
                    mean_fz = bota_dashboard.last_mean_fz if hasattr(bota_dashboard, 'last_mean_fz') else 0.0
                    peak_intensity = bota_dashboard.last_mean_intensity if hasattr(bota_dashboard, 'last_mean_intensity') else 0.0
                    
                    # Save force data to JSON
                    save_force_data_to_json(recording_timestamp, mean_fz, peak_intensity, deformation_mesh, shear_mesh)

                    print("✅ Mesh metadata updated for dashboard.")
                    
                    recorded_deformation_frames.clear()
                    recorded_shear_frames.clear()
                else:
                    print("Warning: Recording finished, but no frames were captured.")
            
            was_recording = is_recording_now

            if time.time() - start_time > 1.0:
                fps = frame_num / (time.time() - start_time)
                daimon_status['fps'] = f"{fps:.2f}"
                print("Output FPS is: {:.2f}".format(fps), end='\r')
                frame_num = 0.0
                start_time = time.time()
    finally:
        bota_dashboard.running = False
        daimon_status['connection'] = 'Disconnected'
        sensor.disconnect()
        cv2.destroyAllWindows()