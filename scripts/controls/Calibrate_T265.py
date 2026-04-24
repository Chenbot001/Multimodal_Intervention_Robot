#!/usr/bin/env python3
"""
Calibrate_T265.py — Calibrate the T265→UR camera-mounting alignment.

The full camera→robot mapping is decomposed into exactly two 3×3 rotations:

    t265_to_ur_align  =  operator_to_ur  @  cam_to_operator

┌──────────────────┬──────────────────────────────────────────────────────────┐
│  cam_to_operator │ How the camera is physically mounted relative to the      │
│                  │ operator's natural perspective.                           │
│                  │ SOLVED HERE from: prompted direction vs T265 body delta  │
├──────────────────┼──────────────────────────────────────────────────────────┤
│  operator_to_ur  │ Maps operator perspective directions to UR base frame.   │
│                  │ FIXED — loaded from config/ur_robot.yaml.                │
│                  │ Only change manually if robot orientation changes.        │
├──────────────────┼──────────────────────────────────────────────────────────┤
│ t265_to_ur_align │ Combined product used by all control scripts.             │
│                  │ Auto-computed: operator_to_ur @ cam_to_operator          │
└──────────────────┴──────────────────────────────────────────────────────────┘

Why operator_to_ur is NOT re-solved here
-----------------------------------------
operator_to_ur encodes where the robot physically goes (in UR base frame)
when the operator moves in their intuitive direction.  This mapping depends
only on the robot's physical orientation in the workspace — it does NOT change
when the camera is remounted.  Re-solving it from operator observations leads
to inconsistent results because the ROBOT_DIRS key labels are defined in UR
convention, but operators perceive directions in their own spatial frame; the
two do not necessarily match, causing Kabsch to compute the wrong rotation.

Calibration pipeline (per step)
--------------------------------
  1. You are prompted to move the camera in your perspective direction (e.g. RIGHT).
  2. Press ENTER — clutch engages: T265 is recorded AND the robot follows the camera.
  3. Move the camera ~5 cm in YOUR prompted direction.
  4. Press ENTER — clutch disengages. Script shows which T265 axis dominated.

The PROMPTED direction is used as ground truth:
  cam_to_operator  — kabsch(measured cam body delta  →  prompted op direction)

cam_to_operator is snapped to {-1, 0, 1} after SVD since camera mounts are
always 90° steps.

Operator perspective convention:  +X right,  +Y up,  -Z forward
UR base convention:                +X forward, +Y left, +Z up

Usage (run from project root):
  python scripts/Calibrate_T265.py

Controls:
  ENTER   — engage / disengage clutch per step
  Ctrl+C  — abort without saving
"""

import os
import re
import sys
import time
import threading

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

try:
    import pyrealsense2 as rs
    RS_AVAILABLE = True
except ImportError:
    print("Error: pyrealsense2 not available. Install with: pip install pyrealsense2")
    RS_AVAILABLE = False

from src.core.hardware_manager import URRobotManager
from src.core.config_loader import UR_CONFIG

CONFIG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'config', 'ur_robot.yaml')
)

# ─── Servo constants (from config) ────────────────────────────────────────────

_T265_CFG   = UR_CONFIG["t265_teleop"]
SERVO_SPEED = _T265_CFG["servo_speed"]
SERVO_ACCEL = _T265_CFG["servo_accel"]
SERVO_DT    = _T265_CFG["servo_dt"]
SERVO_LH    = _T265_CFG["servo_lookahead"]
SERVO_GAIN  = UR_CONFIG["gain"]
TRANS_SCALE = _T265_CFG["translation_scale"]

# ─── Calibration prompts ──────────────────────────────────────────────────────
# Operator-perspective unit vectors the operator will move along.
# Operator convention: +X right, +Y up, -Z forward
CALIB_STEPS = [
    ("RIGHT",   np.array([ 1.,  0.,  0.])),   # operator +X
    ("UP",      np.array([ 0.,  1.,  0.])),   # operator +Y
    ("FORWARD", np.array([ 0.,  0., -1.])),   # operator -Z
]

# UR base convention label map — used in the verification display only
UR_LABELS_CALIB = {
    (0,  1): "forward",  (0, -1): "backward",
    (1,  1): "left",     (1, -1): "right",
    (2,  1): "up",       (2, -1): "down",
}

# ─── Quaternion helper ─────────────────────────────────────────────────────────

def _quat_to_matrix(qx, qy, qz, qw) -> np.ndarray:
    """3×3 rotation matrix from a unit quaternion (x, y, z, w)."""
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])

# ─── SVD-based best-fit rotation ─────────────────────────────────────────────

def kabsch(source_vecs: list, target_vecs: list) -> np.ndarray:
    """
    Find the best-fit proper rotation R s.t. R @ source_vecs[i] ≈ target_vecs[i].
    H = Σ outer(target, source), SVD(H) → U S Vᵀ, R = U Vᵀ  (det-corrected).
    """
    H = sum(np.outer(tv, sv) for sv, tv in zip(source_vecs, target_vecs))
    U, _, Vt = np.linalg.svd(H)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def build_4x4(R3: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R3
    return M


# ─── YAML writer ─────────────────────────────────────────────────────────────

def _fmt_val(v: float):
    return int(v) if abs(v - round(v)) < 1e-9 else round(float(v), 6)


def _matrix_rows(matrix: np.ndarray, indent: int = 4) -> str:
    """Format a 2-D matrix as YAML flow-list rows with `indent` spaces."""
    pad = " " * indent
    lines = []
    for row in matrix:
        vals = [_fmt_val(v) for v in row]
        parts = [f" {v}" if isinstance(v, int) and v >= 0 else str(v) for v in vals]
        lines.append(f"{pad}- [{', '.join(parts)}]")
    return "\n".join(lines)


def _replace_block(content: str, key: str, new_rows: str, indent: int = 2) -> str:
    """Replace a yaml list block (key: \\n  - ...) in-place."""
    pad = " " * indent
    pattern = re.compile(
        rf'{re.escape(pad)}{re.escape(key)}:\n(?:{re.escape(pad)}  - \[.*?\]\n?)*',
        re.MULTILINE,
    )
    replacement = f"{pad}{key}:\n{new_rows}\n"
    new_content, n = pattern.subn(replacement, content)
    if n != 1:
        raise ValueError(f"Expected exactly 1 '{key}' block in YAML, found {n}.")
    return new_content


def write_matrices_to_yaml(
    path: str,
    cam_to_op: np.ndarray,
    combined_4x4: np.ndarray,
) -> None:
    """Write cam_to_operator and t265_to_ur_align to the YAML file.

    operator_to_ur is intentionally NOT written — it is fixed by the
    robot's physical workspace orientation and must be changed manually.
    """
    with open(path) as f:
        content = f.read()

    content = _replace_block(content, "cam_to_operator", _matrix_rows(cam_to_op))
    content = _replace_block(
        content, "t265_to_ur_align", _matrix_rows(combined_4x4), indent=2
    )

    with open(path, "w") as f:
        f.write(content)


# ─── Combined T265 recording + UR servo thread ───────────────────────────────

class CalibCaptureThread(threading.Thread):
    """
    Background thread that simultaneously:
      - Streams T265 pose frames into a sample buffer (for cam_to_operator solve)
      - Drives the UR robot via servoL (so the operator can observe robot motion)

    The servo uses whatever t265_to_ur_align is currently in the YAML — even if
    that mapping is wrong, the operator just reports what they see and the
    calibration corrects both matrices.

    Call start_capture() / stop_capture() around each recording window.
    """

    def __init__(self, pipeline, rtde_r, rtde_c, align_4x4: np.ndarray):
        super().__init__(daemon=True)
        self.pipeline    = pipeline
        self.rtde_r      = rtde_r
        self.rtde_c      = rtde_c
        self.align       = align_4x4
        self._stop_event = threading.Event()
        self._capturing  = False
        self._samples    = []
        self._ref_cam    = None
        self._ref_ur     = None
        self._R_WC_start = None   # camera body orientation (R_WC) at capture start
        self._lock       = threading.Lock()

    def stop(self):
        self._stop_event.set()

    def start_capture(self):
        with self._lock:
            self._samples    = []
            self._ref_cam    = None
            self._ref_ur     = None
            self._R_WC_start = None
            self._capturing  = True

    def stop_capture(self) -> tuple:
        """Returns (position_samples, R_WC_at_start)."""
        with self._lock:
            self._capturing = False
            R = self._R_WC_start if self._R_WC_start is not None else np.eye(3)
            return list(self._samples), R

    def run(self):
        while not self._stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
            except Exception:
                continue
            pf = frames.get_pose_frame()
            if not pf:
                continue
            pd = pf.get_pose_data()
            t = pd.translation
            cur_cam = np.array([t.x, t.y, t.z])

            servo_target = None
            with self._lock:
                if self._capturing:
                    self._samples.append(cur_cam.copy())
                    # Capture reference poses on the very first frame
                    if self._ref_cam is None:
                        self._ref_cam = cur_cam.copy()
                        r = pd.rotation
                        self._R_WC_start = _quat_to_matrix(r.x, r.y, r.z, r.w)
                        try:
                            self._ref_ur = np.array(self.rtde_r.getActualTCPPose())
                        except Exception:
                            self._ref_ur = None
                    if self._ref_ur is not None:
                        # Use body-frame delta for servo drive (same correction as control)
                        delta_world = cur_cam - self._ref_cam
                        delta_cam   = self._R_WC_start.T @ delta_world
                        delta_ur    = self.align[:3, :3] @ delta_cam * TRANS_SCALE
                        servo_target = self._ref_ur.copy()
                        servo_target[:3] += delta_ur
                else:
                    self._ref_cam = None
                    self._ref_ur  = None

            # servoL call is outside the lock to avoid blocking stop_capture()
            if servo_target is not None:
                try:
                    self.rtde_c.servoL(
                        servo_target.tolist(),
                        SERVO_SPEED, SERVO_ACCEL,
                        SERVO_DT, SERVO_LH, SERVO_GAIN,
                    )
                except Exception:
                    pass



# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    if not RS_AVAILABLE:
        print("Error: pyrealsense2 not available.")
        return

    print("=" * 62)
    print("  T265 → UR Camera-Mounting Calibration")
    print("=" * 62)
    print("""
This script solves cam_to_operator ONLY.
operator_to_ur is loaded from the YAML and kept fixed.

For each of 3 steps you will be prompted to move the camera in
YOUR OWN perspective direction (e.g. RIGHT).

  1. Press ENTER to engage the clutch.
  2. Move the camera ~5 cm in YOUR prompted direction.
     Aim for a single, clean axis.
  3. Press ENTER to disengage the clutch.

The prompted direction is used as ground truth.
Slight off-axis camera movement does not matter — SVD + rounding
snaps everything to the nearest 90° rotation.

Press Ctrl+C at any time to abort without saving.
""")

    # ── Load YAML ─────────────────────────────────────────────────────
    with open(CONFIG_PATH) as f:
        yaml_data = yaml.safe_load(f)
    cfg = yaml_data["t265_teleop"]

    # operator_to_ur is fixed — loaded once and never modified by this script.
    op_to_ur  = np.array(cfg["operator_to_ur"], dtype=float)
    # Current matrices used for deviation display and initial servo drive
    old_c2o   = np.array(cfg["cam_to_operator"], dtype=float)
    align_4x4 = np.array(cfg["t265_to_ur_align"], dtype=float)

    # ── Connect robot ─────────────────────────────────────────────────
    ur = URRobotManager()
    print("Connecting to UR robot...", end=" ", flush=True)
    ur.connect()
    if not ur.connected:
        print("FAILED. Exiting.")
        return
    print("OK")

    # ── Move to init_pose ─────────────────────────────────────────────
    init_pose  = yaml_data["init_pose"]
    init_speed = yaml_data["init_move_speed"]
    init_accel = yaml_data["init_move_accel"]
    print(f"Moving to init_pose {init_pose} deg...", end=" ", flush=True)
    try:
        ur.rtde_c.moveJ(
            [np.deg2rad(v) for v in init_pose],
            init_speed,
            init_accel,
        )
        print("OK")
    except Exception as e:
        print(f"FAILED: {e}")
        ur.disconnect()
        return

    # ── Start T265 pipeline ───────────────────────────────────────────
    print("Starting T265 pipeline...", end=" ", flush=True)
    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.pose)
    try:
        pipeline.start(rs_cfg)
    except Exception as e:
        print(f"FAILED: {e}")
        ur.disconnect()
        return
    print("OK")
    time.sleep(1.0)   # let the tracker settle

    capture = CalibCaptureThread(pipeline, ur.rtde_r, ur.rtde_c, align_4x4)
    capture.start()

    # ── Helper labels ─────────────────────────────────────────────────
    # Operator-perspective labels for deviation display
    OP_LABELS = {
        (0,  1): "right",   (0, -1): "left",
        (1,  1): "up",      (1, -1): "down",
        (2, -1): "forward", (2,  1): "backward",
    }

    def dominant_op_label(vec):
        ax = int(np.argmax(np.abs(vec)))
        sign = 1 if vec[ax] >= 0 else -1
        return OP_LABELS.get((ax, sign), "?")

    # ── Recording loop ────────────────────────────────────────────────
    # Per step we collect two aligned vectors:
    #   cam_vecs   — measured T265 unit delta (raw camera body frame)
    #   op_vecs    — prompted operator direction (ground truth)
    cam_vecs   = []
    op_vecs    = []

    def shutdown(msg):
        print(msg)
        capture.stop()
        try:
            ur.rtde_c.servoStop()
        except Exception:
            pass
        pipeline.stop()
        capture.join(timeout=2.0)
        ur.disconnect()

    try:
        step = 1
        while step <= len(CALIB_STEPS):
            label, op_vec = CALIB_STEPS[step - 1]

            print(f"\n{'─' * 54}")
            print(f"  Step {step}/{len(CALIB_STEPS)}  ─  Move camera YOUR {label}")
            print(f"  Keep the motion ~5 cm and as straight as possible.")

            input(f"\n  Press ENTER to ENGAGE clutch and start moving...")
            capture.start_capture()
            print(f"  [ENGAGED — move {label} now, watch robot]")

            input(f"  Press ENTER to DISENGAGE clutch when done...")
            samples, R_WC_start = capture.stop_capture()
            try:
                ur.rtde_c.servoStop()
            except Exception:
                pass
            print(f"  [DISENGAGED — {len(samples)} frames captured]")

            # ── Validate recording ────────────────────────────────────
            if len(samples) < 5:
                print("  Too few frames — hold still first, then try again.")
                continue   # retry same step

            pts = np.array(samples)            # (N, 3) world frame
            increments_world = np.diff(pts, axis=0)
            # Convert increments to camera body frame using orientation at capture start.
            # This makes axis detection invariant to T265 world frame initialisation.
            increments_body = increments_world @ R_WC_start   # == (R_WC_start.T @ inc.T).T

            # Axis activity = cumulative absolute travel per body axis.
            axis_activity = np.sum(np.abs(increments_body), axis=0)  # (3,)
            dominant_axis = int(np.argmax(axis_activity))

            # Sign from net body-frame displacement
            net_body = R_WC_start.T @ (pts[-1] - pts[0])
            sign = 1.0 if net_body[dominant_axis] >= 0 else -1.0

            # Unit vector along the dominant axis only
            cam_unit = np.zeros(3)
            cam_unit[dominant_axis] = sign

            total_travel = float(axis_activity[dominant_axis])
            if total_travel < 0.005:
                print(f"  Motion too small ({total_travel*1000:.1f} mm cumulative). "
                       "Move further (~5 cm) and try again.")
                continue   # retry same step

            # ── Deviation display (info only) ─────────────────────────
            # Map raw T265 delta to operator frame using the OLD cam_to_operator
            # and show how close it is to the prompted direction.
            op_mapped = old_c2o @ cam_unit
            observed_op_lbl = dominant_op_label(op_mapped)
            agreement = float(np.dot(op_mapped, op_vec))
            print(f"\n  Axis activity (|Δx|,|Δy|,|Δz|) : {axis_activity}")
            print(f"  Dominant camera axis           : {'XYZ'[dominant_axis]}  "
                  f"(sign {'+'if sign>0 else '-'}, travel = {total_travel*100:.1f} cm)")
            print(f"  Mapped to op frame             : dominant = {observed_op_lbl}  "
                  f"(agreement with '{label.lower()}': {agreement:+.2f})")
            if agreement < 0.7:
                print(f"  NOTE: camera motion deviated from {label.lower()} "
                       "— prompted direction used as ground truth.")

            cam_vecs.append(cam_unit)
            op_vecs.append(op_vec)
            step += 1

    except KeyboardInterrupt:
        shutdown("\n\nAborted — no changes written.")
        return

    shutdown("")

    # ── Solve cam_to_operator only ────────────────────────────────────
    #
    #   cam_to_operator:  R s.t. R @ cam_unit[i]  ≈  op_vec[i]  (prompted)
    #   operator_to_ur:   UNCHANGED — loaded from config/ur_robot.yaml
    #
    print(f"\n{'=' * 54}")
    print("Computing matrices...\n")

    cam_to_op_raw = kabsch(cam_vecs, op_vecs)

    # Snap to {-1, 0, 1} — camera mounts are always 90° steps
    cam_to_op    = np.round(cam_to_op_raw).astype(float)
    combined     = np.round(op_to_ur @ cam_to_op).astype(float)
    combined_4x4 = build_4x4(combined)

    print("cam_to_operator (solved, rounded):")
    for row in cam_to_op:
        print("  [" + "  ".join(f"{v:5.0f}" for v in row) + " ]")

    print("\noperator_to_ur (unchanged from yaml):")
    for row in op_to_ur:
        print("  [" + "  ".join(f"{v:5.0f}" for v in row) + " ]")

    print("\nt265_to_ur_align = operator_to_ur @ cam_to_operator (rounded):")
    for row in combined:
        print("  [" + "  ".join(f"{v:5.0f}" for v in row) + " ]")

    # Pre-rounding residual check on cam_to_operator
    res = float(np.max(np.abs(cam_to_op_raw @ cam_to_op_raw.T - np.eye(3))))
    print()
    if res > 0.15:
        print(f"  WARNING (cam_to_operator): residual = {res:.4f}  "
               "— movements may have been off-axis.")
    else:
        print(f"  Quality OK (cam_to_operator, residual = {res:.6f})")

    # ── Verification table ────────────────────────────────────────────
    def dominant_ur_label(vec):
        ax = int(np.argmax(np.abs(vec)))
        sign = 1 if vec[ax] >= 0 else -1
        return UR_LABELS_CALIB.get((ax, sign), "?")

    print("\nVerification — full pipeline (camera axis → robot direction):")
    cam_test_axes = [
        ("+X (cam right)",  np.array([ 1.,  0.,  0.])),
        ("+Y (cam up)",     np.array([ 0.,  1.,  0.])),
        ("-Z (cam fwd)",    np.array([ 0.,  0., -1.])),
    ]
    for lbl, v in cam_test_axes:
        op_out  = cam_to_op @ v
        ur_out  = op_to_ur  @ op_out
        print(f"  T265 {lbl:16s}  →  op {dominant_op_label(op_out):9s}  "
              f"→  robot {dominant_ur_label(ur_out)}")

    # ── Persist ───────────────────────────────────────────────────────
    print()
    confirm = input("Write updated matrices to config/ur_robot.yaml? [y/N]: ").strip().lower()
    if confirm == "y":
        write_matrices_to_yaml(CONFIG_PATH, cam_to_op, combined_4x4)
        print(f"\nWrote cam_to_operator + t265_to_ur_align to {CONFIG_PATH}")
        print("(operator_to_ur was NOT modified — edit manually if robot orientation changes.)")
        print("Run scripts/tests/test_t265.py to verify translation perspective.")
    else:
        print("Discarded — no changes written.")

    print("\nCalibration complete.")


if __name__ == "__main__":
    main()
