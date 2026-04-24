#!/usr/bin/env python3
"""
test_t265.py

Verify T265 camera perspective alignment by recording short motion sessions
and reporting the dominant direction in both the operator frame and the
UR-mapped frame.

Usage:
    python scripts/tests/test_t265.py

Controls:
  ENTER     - start / stop a recording session
  Ctrl+C    - quit
"""

import os
import sys
import time
from typing import Optional

import numpy as np
from pynput import keyboard

PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..')
)
sys.path.insert(0, PROJECT_ROOT)

try:
    import pyrealsense2 as rs
    RS_AVAILABLE = True
except ImportError:
    print("Error: pyrealsense2 not available. Install with: pip install pyrealsense2")
    RS_AVAILABLE = False

from src.core.config_loader import UR_CONFIG


T265_TO_UR_ALIGN = np.array(
    UR_CONFIG["t265_teleop"]["t265_to_ur_align"], dtype=float
)
CAMERA_TO_OPERATOR_ALIGN = np.array(
    UR_CONFIG["t265_teleop"]["cam_to_operator"], dtype=float
)

# ─── Keyboard state ───────────────────────────────────────────────────────────

enter_pressed = False

def on_press(key):
    global enter_pressed
    if key == keyboard.Key.enter:
        enter_pressed = True


# ─── Direction helpers ────────────────────────────────────────────────────────

OP_LABELS = {
    (0,  1): "right",    (0, -1): "left",
    (1,  1): "up",       (1, -1): "down",
    (2, -1): "forward",  (2,  1): "backward",
}
UR_LABELS = {
    (0,  1): "forward",  (0, -1): "backward",
    (1,  1): "left",     (1, -1): "right",
    (2,  1): "up",       (2, -1): "down",
}

def _quat_to_matrix(qx, qy, qz, qw) -> np.ndarray:
    """Build rotation matrix R_WC (camera body -> world) from a unit quaternion."""
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


def dominant_label(vec, labels):
    ax = int(np.argmax(np.abs(vec)))
    sign = 1 if vec[ax] >= 0 else -1
    return labels.get((ax, sign), "?"), ax, sign


def analyse_samples(samples: list, R_WC_start: np.ndarray) -> Optional[dict]:
    """
    Given a list of T265 translation samples (world frame) and the camera
    body orientation at recording start, compute the dominant camera body
    axis using cumulative per-frame absolute deltas, then map into operator
    and UR frames.

    Returns None if the motion is too small or there are too few samples.
    """
    if len(samples) < 5:
        return None

    pts = np.array(samples)                          # (N, 3) world frame
    increments_world = np.diff(pts, axis=0)          # (N-1, 3)
    increments_body  = increments_world @ R_WC_start # convert to camera body frame
    axis_activity = np.sum(np.abs(increments_body), axis=0)   # (3,)

    dominant_axis = int(np.argmax(axis_activity))
    total_travel = float(axis_activity[dominant_axis])

    if total_travel < 0.005:
        return None

    net_body = R_WC_start.T @ (pts[-1] - pts[0])
    sign = 1.0 if net_body[dominant_axis] >= 0 else -1.0

    cam_unit = np.zeros(3)
    cam_unit[dominant_axis] = sign

    op_vec  = CAMERA_TO_OPERATOR_ALIGN @ cam_unit
    ur_vec  = T265_TO_UR_ALIGN[:3, :3] @ cam_unit

    op_lbl, op_ax, op_sign = dominant_label(op_vec, OP_LABELS)
    ur_lbl, ur_ax, ur_sign = dominant_label(ur_vec, UR_LABELS)

    # Purity: fraction of total travel on the dominant axis (1.0 = perfectly straight).
    total_all = float(np.sum(axis_activity))
    purity = total_travel / total_all if total_all > 0 else 0.0
    # Angle deviation from a pure axis (0° = perfect, >20° = noisy).
    angle_deg = float(np.degrees(np.arccos(np.clip(purity, 0.0, 1.0))))

    return {
        "axis_activity": axis_activity,
        "dominant_axis": dominant_axis,
        "sign": sign,
        "total_travel_cm": total_travel * 100,
        "purity": purity,
        "angle_deg": angle_deg,
        "cam_unit": cam_unit,
        "op_vec": op_vec,
        "ur_vec": ur_vec,
        "op_label": op_lbl,
        "ur_label": ur_lbl,
        "n_frames": len(samples),
    }


def print_result(r: dict) -> None:
    ax_str = "  ".join(
        f"{'XYZ'[i]}={r['axis_activity'][i]*100:.1f}cm" for i in range(3)
    )
    purity_pct = r['purity'] * 100
    purity_note = ("  ✓ clean" if purity_pct >= 80
                   else "  ⚠ off-axis (T265 world frame tilt or curved motion)")
    print("\n" + "=" * 62)
    print(f"  Frames recorded    : {r['n_frames']}")
    print(f"  Axis activity      : {ax_str}")
    print(f"  Dominant cam axis  : {'XYZ'[r['dominant_axis']]}  "
          f"({'+'if r['sign']>0 else '-'})  "
          f"total = {r['total_travel_cm']:.1f} cm")
    print(f"  Axis purity        : {purity_pct:.0f}%  "
          f"({r['angle_deg']:.1f}° from pure axis){purity_note}")
    print()
    print(f"  Operator frame     : {r['op_label']:>9s}   "
          f"vec = [{', '.join(f'{v:+.2f}' for v in r['op_vec'])}]")
    print(f"  UR-mapped frame    : {r['ur_label']:>9s}   "
          f"vec = [{', '.join(f'{v:+.2f}' for v in r['ur_vec'])}]")
    print("=" * 62)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    global enter_pressed

    if not RS_AVAILABLE:
        print("Required libraries unavailable. Exiting.")
        return

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.pose)

    try:
        pipeline.start(rs_config)
    except Exception as e:
        print(f"Failed to start T265 pipeline: {e}")
        listener.stop()
        return

    print("T265 pipeline started.")
    print("Press ENTER to start a recording. Press ENTER again to stop and see result.")
    print("Ctrl+C to quit.\n")
    time.sleep(1.0)

    recording    = False
    samples      = []
    R_WC_start   = np.eye(3)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue

            pose_data = pose_frame.get_pose_data()
            t = pose_data.translation
            current_t = np.array([t.x, t.y, t.z])

            # Handle ENTER toggle
            if enter_pressed:
                enter_pressed = False
                if not recording:
                    samples    = []
                    R_WC_start = np.eye(3)  # will be set on first captured frame
                    recording  = True
                    print("[RECORDING — move camera along one axis, then press ENTER]")
                else:
                    recording = False
                    result = analyse_samples(samples, R_WC_start)
                    if result is None:
                        if len(samples) < 5:
                            print("\n  Too few frames captured. Try again.")
                        else:
                            print("\n  Motion too small (<5 mm). Move further and try again.")
                    else:
                        print_result(result)
                    print("\nPress ENTER to start a new recording. Ctrl+C to quit.")

            if recording:
                if len(samples) == 0:
                    # Capture camera body orientation at the first recorded frame
                    rot = pose_data.rotation
                    R_WC_start = _quat_to_matrix(rot.x, rot.y, rot.z, rot.w)
                samples.append(current_t)
                # Live frame counter on same line
                print(f"\r  {len(samples)} frames...", end="", flush=True)

    except KeyboardInterrupt:
        print("\nStopping test...")
    finally:
        pipeline.stop()
        listener.stop()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()