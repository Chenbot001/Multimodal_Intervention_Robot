#!/usr/bin/env python3
"""
T265-based teleoperation for the UR robot arm.

Uses an Intel RealSense T265 tracking camera to mirror camera motion onto the
UR arm TCP in real time. A clutch (SPACE key) lets the operator engage and
disengage tracking without losing the robot's current pose.

Controls:
  SPACE   — toggle clutch (engage / disengage tracking)
  Ctrl+C  — graceful shutdown

Usage (run from project root):
  python scripts/UR_Control_T265.py
"""

import os
import sys
import time
import numpy as np
from pynput import keyboard

# Ensure project root is on the path so src.core.* imports resolve
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

try:
    import pyrealsense2 as rs
    RS_AVAILABLE = True
except ImportError:
    print("Error: pyrealsense2 not available. Install with: pip install pyrealsense2")
    RS_AVAILABLE = False

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    print("Error: scipy not available. Install with: pip install scipy")
    SCIPY_AVAILABLE = False

from src.core.hardware_manager import URRobotManager
from src.core.config_loader import UR_CONFIG

# ============================================================================
# T265 teleop tuning parameters
# These are distinct from the keyboard-step servo params in CONFIG because the
# T265 loop runs at ~500 Hz and requires tighter lookahead / lower dt.
# ============================================================================

# Scale T265 translation before applying to UR workspace (1.0 = 1:1 mapping)
TRANSLATION_SCALE = 1.0

# servoL parameters for the high-frequency T265 tracking loop
SERVO_SPEED      = 0.5    # m/s
SERVO_ACCEL      = 0.5    # m/s²
SERVO_DT         = 0.002  # seconds  →  ~500 Hz control loop
SERVO_LOOKAHEAD  = 0.1    # seconds  (tighter than keyboard-step 0.2 s)
SERVO_GAIN       = UR_CONFIG["gain"]  # shared with keyboard mode (300)

# Coordinate alignment matrix: maps T265 camera frame → UR robot base frame
# Loaded from config/ur_robot.yaml  →  t265_teleop.t265_to_ur_align
T265_TO_UR_ALIGN = np.array(
    UR_CONFIG["t265_teleop"]["t265_to_ur_align"], dtype=float
)


# ============================================================================
# Clutch state  (toggled by SPACE key via pynput callback)
# ============================================================================
clutch_active = False


def on_press(key):
    global clutch_active
    try:
        if key == keyboard.Key.space:
            clutch_active = not clutch_active
    except AttributeError:
        pass


# ============================================================================
# Pose math utilities  (same conventions as test_t265_ur.py)
# ============================================================================

def create_pose_matrix(translation, rotation_quat) -> np.ndarray:
    """Build a 4×4 homogeneous transform from a T265 pose frame."""
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat([
        rotation_quat.x, rotation_quat.y,
        rotation_quat.z, rotation_quat.w
    ]).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def matrix_to_pose_vector(matrix: np.ndarray) -> list:
    """Convert a 4×4 homogeneous matrix to a UR pose vector [x, y, z, rx, ry, rz]."""
    pos = matrix[:3, 3]
    rot = R.from_matrix(matrix[:3, :3]).as_rotvec()
    return [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]


def pose_vector_to_matrix(pose_vec) -> np.ndarray:
    """Convert a UR pose vector [x, y, z, rx, ry, rz] to a 4×4 homogeneous matrix."""
    matrix = np.eye(4)
    matrix[:3, 3] = pose_vec[:3]
    matrix[:3, :3] = R.from_rotvec(pose_vec[3:]).as_matrix()
    return matrix


# ============================================================================
# Main
# ============================================================================

def main():
    global clutch_active

    if not RS_AVAILABLE or not SCIPY_AVAILABLE:
        print("Required libraries unavailable. Exiting.")
        return

    # ------------------------------------------------------------------
    # Hardware initialisation — UR robot only
    # ------------------------------------------------------------------
    ur_robot = URRobotManager()
    ur_robot.connect()

    if not ur_robot.connected:
        print("UR robot not connected. Exiting.")
        return

    rtde_r = ur_robot.rtde_r
    rtde_c = ur_robot.rtde_c

    # ------------------------------------------------------------------
    # Move to initial joint configuration
    # ------------------------------------------------------------------
    init_pose  = UR_CONFIG["init_pose"]
    init_speed = UR_CONFIG["init_move_speed"]
    init_accel = UR_CONFIG["init_move_accel"]
    # Joint angles are stored in degrees in the YAML; convert to radians.
    init_joints_rad = [np.deg2rad(v) for v in init_pose]
    print(f"[Init] Moving to init joints (deg): {init_pose}")
    try:
        rtde_c.moveJ(init_joints_rad, init_speed, init_accel)
        print("[Init] Reached init pose.")
    except Exception as e:
        print(f"[Init] moveJ to init pose failed: {e}")
        ur_robot.disconnect()
        return

    # ------------------------------------------------------------------
    # T265 pipeline
    # ------------------------------------------------------------------
    pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.pose)

    try:
        pipeline.start(rs_config)
        print("T265 pipeline started.")
    except Exception as e:
        print(f"Failed to start T265 pipeline: {e}")
        ur_robot.disconnect()
        return

    # ------------------------------------------------------------------
    # Keyboard listener (clutch toggle)
    # ------------------------------------------------------------------
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Keyboard listener started.")

    # ------------------------------------------------------------------
    # Startup calibration: lock the T265 baseline to the current UR pose
    # so subsequent deltas are applied relative to the arm's real position.
    # ------------------------------------------------------------------
    time.sleep(1.0)  # Let the T265 stream stabilise

    print("\n[Calibration] Reading initial poses to align T265 and UR frames...")
    frames = pipeline.wait_for_frames()
    pose_frame = frames.get_pose_frame()

    if pose_frame:
        pose_data = pose_frame.get_pose_data()
        base_t265_matrix = create_pose_matrix(pose_data.translation, pose_data.rotation)
        base_ur_matrix = pose_vector_to_matrix(rtde_r.getActualTCPPose())
        print("[Calibration] Done. Arm and camera frames locked.")
        print("\n>>> System ready. Press SPACE to engage clutch and start tracking. <<<\n")
    else:
        print("[Calibration] Warning: no T265 frame received — using identity matrices.")
        base_t265_matrix = np.eye(4)
        base_ur_matrix = np.eye(4)

    # Per-clutch-engagement reference poses (set on each rising edge)
    clutch_t265_matrix = None
    clutch_ur_matrix   = None
    R_clutch_WC        = None   # Camera body orientation (world frame) at clutch engage
    was_clutch_active  = False

    # ------------------------------------------------------------------
    # Control loop  (~500 Hz, gated by T265 frame arrival)
    # ------------------------------------------------------------------
    try:
        while True:
            frames = pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue

            pose_data = pose_frame.get_pose_data()

            # ── World frame reset block removed ──────────────────────────────
            # Translation is now converted to camera body frame at clutch engage
            # (see below), making it invariant to T265 world frame initialisation.

            if clutch_active:
                # Clutch rising edge: capture reference poses for this engagement
                if not was_clutch_active:
                    clutch_t265_matrix = create_pose_matrix(
                        pose_data.translation, pose_data.rotation
                    )
                    clutch_ur_matrix = pose_vector_to_matrix(rtde_r.getActualTCPPose())
                    # Camera body orientation at clutch engage (R_WC: world←body).
                    # R_WC.T converts world-frame deltas → camera body frame,
                    # making translation control invariant to T265 startup tilt.
                    R_clutch_WC = clutch_t265_matrix[:3, :3]
                    was_clutch_active = True
                    print("\n>>> Clutch ENGAGED — tracking active. Press SPACE to disengage. <<<")

                current_t265_matrix = create_pose_matrix(
                    pose_data.translation, pose_data.rotation
                )

                # Rotation delta since clutch engagement, expressed in T265 camera frame.
                # Using the clutch reference (not script-startup base) means each
                # engagement is independent and inter-clutch drift does not accumulate.
                R_delta_cam = (
                    clutch_t265_matrix[:3, :3].T @ current_t265_matrix[:3, :3]
                )

                # Transform the delta into the TCP frame at clutch engagement.
                #   R_cam_to_tcp = R_tcp_in_base.T  @  R_cam_in_base
                #                = clutch_ur_matrix.T  @  T265_TO_UR_ALIGN[:3,:3]
                # This accounts for both the current arm configuration and the
                # physical camera mounting (including the j5 = −45° EEF offset),
                # so rotations happen around the TCP's own local axes.
                R_cam_to_tcp = (
                    clutch_ur_matrix[:3, :3].T @ T265_TO_UR_ALIGN[:3, :3]
                )
                R_delta_tcp = R_cam_to_tcp @ R_delta_cam @ R_cam_to_tcp.T

                # Right-multiply: apply the delta in TCP-local axes.
                target_rotation = clutch_ur_matrix[:3, :3] @ R_delta_tcp

                # Translation: convert world-frame delta to camera body frame at
                # clutch engage, then apply T265_TO_UR_ALIGN (calibrated in body frame).
                trans_delta_world = (
                    current_t265_matrix[:3, 3] - clutch_t265_matrix[:3, 3]
                )
                trans_delta_body = R_clutch_WC.T @ trans_delta_world
                mapped_trans_delta = T265_TO_UR_ALIGN[:3, :3] @ trans_delta_body
                mapped_trans_delta *= TRANSLATION_SCALE

                # Compose final target pose matrix
                target_ur_matrix = np.eye(4)
                target_ur_matrix[:3, :3] = target_rotation
                target_ur_matrix[:3, 3] = clutch_ur_matrix[:3, 3] + mapped_trans_delta

                target_pose_vec = matrix_to_pose_vector(target_ur_matrix)

                try:
                    rtde_c.servoL(
                        target_pose_vec,
                        SERVO_SPEED, SERVO_ACCEL,
                        SERVO_DT, SERVO_LOOKAHEAD, SERVO_GAIN
                    )
                    ur_robot.current_pose = target_pose_vec
                except Exception as e:
                    print(f"servoL error: {e}")

            else:
                if was_clutch_active:
                    try:
                        rtde_c.servoStop()
                    except Exception:
                        pass
                    was_clutch_active = False
                    print("\n>>> Clutch DISENGAGED — tracking paused. Press SPACE to resume. <<<")

            time.sleep(SERVO_DT)

    except KeyboardInterrupt:
        print("\nInterrupt received. Shutting down...")
    finally:
        try:
            rtde_c.servoStop()
        except Exception:
            pass
        pipeline.stop()
        listener.stop()
        ur_robot.disconnect()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
