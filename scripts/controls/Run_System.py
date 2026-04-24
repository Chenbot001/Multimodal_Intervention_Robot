#!/usr/bin/env python3
"""
Run_System.py — Combined VAIT system controller.

Runs both control loops concurrently in separate threads:
  • Intel RealSense T265  →  UR robot arm teleop   (6-DOF TCP tracking)
  • Rotary encoder        →  Stepper motors         (differential grip drive)

Controls
--------
  SPACE  — engage / disengage clutch (UR arm + stepper gripper together)
  Z      — home steppers (→1000 → 0 → 500) and zero encoder reference
  ESC    — graceful shutdown

Usage (run from project root):
  python scripts/Run_System.py
"""

import os
import sys
import time
import signal
import threading

import numpy as np
import serial
import minimalmodbus
from pynput import keyboard

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

try:
    import pyrealsense2 as rs
    RS_AVAILABLE = True
except ImportError:
    print("Warning: pyrealsense2 not available — UR T265 tracking disabled.")
    RS_AVAILABLE = False

try:
    from scipy.spatial.transform import Rotation as ScipyR
    SCIPY_AVAILABLE = True
except ImportError:
    print("Warning: scipy not available — UR T265 tracking disabled.")
    SCIPY_AVAILABLE = False

from src.core.hardware_manager import URRobotManager, StepperMotorManager, find_stepper_port
from src.core.config_loader import UR_CONFIG, GRIPPER_CONFIG, DISPLAY_CONFIG

# ── T265 / UR constants ──────────────────────────────────────────────────────

_T265_CFG        = UR_CONFIG["t265_teleop"]
SERVO_SPEED      = _T265_CFG["servo_speed"]
SERVO_ACCEL      = _T265_CFG["servo_accel"]
SERVO_DT         = _T265_CFG["servo_dt"]
SERVO_LOOKAHEAD  = _T265_CFG["servo_lookahead"]
SERVO_GAIN       = UR_CONFIG["gain"]
TRANS_SCALE      = _T265_CFG["translation_scale"]
T265_TO_UR_ALIGN = np.array(_T265_CFG["t265_to_ur_align"], dtype=float)

# ── Encoder / stepper constants ──────────────────────────────────────────────

STEPS_PER_DEG      = 500.0 / 720.0   # ~0.69 steps per degree
ANGLE_DEADBAND_DEG = 1.0
ENCODER_POLL_HZ    = 10

_ENC_SLAVE  = 1
_ENC_BAUD   = 9600
_ENC_REG    = 0
_ENC_RES    = 2 ** 15                 # encoder counts per revolution

_DISPLAY_INTERVAL = DISPLAY_CONFIG.get("update_interval_seconds", 0.5)


# ============================================================================
# Pose math  (T265 / UR)
# ============================================================================

def _create_pose_matrix(translation, rotation_quat) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = ScipyR.from_quat([
        rotation_quat.x, rotation_quat.y,
        rotation_quat.z, rotation_quat.w,
    ]).as_matrix()
    m[:3, 3] = [translation.x, translation.y, translation.z]
    return m


def _matrix_to_pose_vec(m: np.ndarray) -> list:
    pos = m[:3, 3]
    rot = ScipyR.from_matrix(m[:3, :3]).as_rotvec()
    return [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]


def _pose_vec_to_matrix(v) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3]  = v[:3]
    m[:3, :3] = ScipyR.from_rotvec(v[3:]).as_matrix()
    return m


# ============================================================================
# Encoder helpers
# ============================================================================

def _make_enc_instrument(port: str) -> minimalmodbus.Instrument:
    inst = minimalmodbus.Instrument(port, _ENC_SLAVE)
    inst.serial.baudrate = _ENC_BAUD
    inst.serial.bytesize = 8
    inst.serial.parity   = minimalmodbus.serial.PARITY_NONE
    inst.serial.stopbits = 1
    inst.serial.timeout  = 0.5
    inst.mode = minimalmodbus.MODE_RTU
    return inst


def _find_encoder_port(skip_port: str) -> str:
    """Return the encoder port from config (auto-detection disabled).
    Update configs/gripper.yaml encoder.port if the port has changed.
    """
    return GRIPPER_CONFIG["encoder"]["port"]


def _read_angle(instrument: minimalmodbus.Instrument):
    """Return continuous multi-turn angle in degrees, or None on read error."""
    try:
        raw         = instrument.read_long(_ENC_REG, 3, False)
        single_turn = raw & 0x7FFF
        turn_count  = raw >> 15
        return turn_count * 360.0 + (single_turn / _ENC_RES) * 360.0
    except minimalmodbus.ModbusException:
        return None


# ============================================================================
# Encoder follow thread  (from Stepper_Control_Encoder.py)
# ============================================================================

class EncoderFollowState:
    def __init__(self):
        self.running        = True
        self._follow_mode   = False
        self._enc_offset    = 0.0
        self._lock          = threading.Lock()

    def get_follow_mode(self) -> bool:
        with self._lock:
            return self._follow_mode

    def set_follow_mode(self, v: bool):
        with self._lock:
            self._follow_mode = v

    def get_offset(self) -> float:
        with self._lock:
            return self._enc_offset

    def set_offset(self, v: float):
        with self._lock:
            self._enc_offset = v


class EncoderFollowThread(threading.Thread):
    def __init__(self, instrument, stepper: StepperMotorManager,
                 enc_state: EncoderFollowState):
        super().__init__(daemon=True, name="EncoderFollow")
        self.instrument = instrument
        self.stepper    = stepper
        self.enc_state  = enc_state
        self._last_cmd_angle = 0.0
        self._overflow       = 0.0
        self._latest_raw     = None
        self._cache_lock     = threading.Lock()

    def run(self):
        dt = 1.0 / ENCODER_POLL_HZ
        while self.enc_state.running:
            raw = _read_angle(self.instrument)
            with self._cache_lock:
                self._latest_raw = raw

            if self.enc_state.get_follow_mode() and self.stepper.connected \
                    and raw is not None:
                relative  = raw - self.enc_state.get_offset()
                effective = self._apply_overflow(relative)
                if abs(effective - self._last_cmd_angle) >= ANGLE_DEADBAND_DEG:
                    self._command_steppers(effective)
                    self._last_cmd_angle = effective
            time.sleep(dt)

    def _apply_overflow(self, relative: float) -> float:
        center    = GRIPPER_CONFIG["stepper"]["initial_pos"]
        max_s     = GRIPPER_CONFIG["stepper"]["max_steps"]
        angle_max =  (max_s - center) / STEPS_PER_DEG
        angle_min = -(center)         / STEPS_PER_DEG
        effective = relative - self._overflow
        if effective > angle_max:
            self._overflow += effective - angle_max
            effective = angle_max
        elif effective < angle_min:
            self._overflow += effective - angle_min
            effective = angle_min
        return effective

    def _command_steppers(self, angle_deg: float):
        center = GRIPPER_CONFIG["stepper"]["initial_pos"]
        max_s  = GRIPPER_CONFIG["stepper"]["max_steps"]
        speed  = GRIPPER_CONFIG["stepper"]["fixed_motor_speed"]
        diff = angle_deg * STEPS_PER_DEG
        m1   = max(0, min(max_s, int(center + diff)))
        m2   = max(0, min(max_s, int(center - diff)))
        self.stepper.send_move_command(m1, m2, speed)

    def reset_tracking(self):
        self._last_cmd_angle = 0.0
        self._overflow       = 0.0

    def set_resume_angle(self, angle: float):
        self._last_cmd_angle = angle
        self._overflow       = 0.0

    def get_latest_raw(self):
        with self._cache_lock:
            return self._latest_raw


# ============================================================================
# T265 control thread  (from UR_Control_T265.py)
# ============================================================================

class T265ControlThread(threading.Thread):
    """
    Background thread running the T265 → UR servoL loop.
    `clutch_active` is a reference to the shared bool held by VAITSystem;
    the GIL makes bool assignment atomic so no explicit lock is needed.
    """

    def __init__(self, rtde_r, rtde_c, pipeline, clutch_ref: list):
        super().__init__(daemon=True, name="T265Control")
        self.rtde_r        = rtde_r
        self.rtde_c        = rtde_c
        self.pipeline      = pipeline
        self._clutch_ref   = clutch_ref   # clutch_ref[0] is the shared bool
        self._stop_event   = threading.Event()
        # Status exposed to display loop
        self.status        = "Calibrating..."
        self.current_pose  = None

    @property
    def clutch_active(self) -> bool:
        return self._clutch_ref[0]

    def stop(self):
        self._stop_event.set()

    def run(self):
        # ── Startup calibration ──────────────────────────────────────
        frames     = self.pipeline.wait_for_frames()
        pose_frame = frames.get_pose_frame()
        if pose_frame:
            pd = pose_frame.get_pose_data()
            # base_* stored but only used for the calibration log message;
            # the control loop is fully per-engagement (clutch reference).
            self.status = "Ready — SPACE to engage clutch"
        else:
            self.status = "Warn: no T265 frame at startup — using identity"

        clutch_t265  = None
        clutch_ur    = None
        R_clutch_WC  = None   # camera body orientation (world←body) at clutch engage
        was_active   = False

        # ── Control loop ──────────────────────────────────────────────
        while not self._stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames()
            except Exception:
                break

            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue

            pd = pose_frame.get_pose_data()

            if self.clutch_active:
                # Rising edge: capture reference poses for this engagement
                if not was_active:
                    clutch_t265 = _create_pose_matrix(pd.translation, pd.rotation)
                    clutch_ur   = _pose_vec_to_matrix(self.rtde_r.getActualTCPPose())
                    # Camera body orientation at clutch engage (R_WC: world←body).
                    # R_WC.T converts world-frame translation deltas → camera body
                    # frame, making control invariant to T265 world frame tilt.
                    R_clutch_WC = clutch_t265[:3, :3]
                    was_active  = True
                    self.status = "ENGAGED"

                cur_t265 = _create_pose_matrix(pd.translation, pd.rotation)

                # Rotation delta since clutch engagement (T265 camera frame)
                R_delta_cam = clutch_t265[:3, :3].T @ cur_t265[:3, :3]

                # Express delta in TCP-local frame — accounts for arm pose and
                # any EEF/camera mounting offset (e.g. j5 = −45°)
                R_cam_to_tcp = clutch_ur[:3, :3].T @ T265_TO_UR_ALIGN[:3, :3]
                R_delta_tcp  = R_cam_to_tcp @ R_delta_cam @ R_cam_to_tcp.T

                # Right-multiply: rotate around TCP-local axes
                target_rot = clutch_ur[:3, :3] @ R_delta_tcp

                # Translation: convert world-frame delta → camera body frame at
                # clutch engage, then apply calibrated alignment (body → UR base).
                trans_delta_world = cur_t265[:3, 3] - clutch_t265[:3, 3]
                trans_delta_body  = R_clutch_WC.T @ trans_delta_world
                trans_delta = T265_TO_UR_ALIGN[:3, :3] @ trans_delta_body
                trans_delta *= TRANS_SCALE

                target = np.eye(4)
                target[:3, :3] = target_rot
                target[:3, 3]  = clutch_ur[:3, 3] + trans_delta

                target_vec = _matrix_to_pose_vec(target)
                try:
                    self.rtde_c.servoL(
                        target_vec,
                        SERVO_SPEED, SERVO_ACCEL,
                        SERVO_DT, SERVO_LOOKAHEAD, SERVO_GAIN,
                    )
                    self.current_pose = target_vec
                except Exception as e:
                    self.status = f"servoL error: {e}"

            else:
                if was_active:
                    try:
                        self.rtde_c.servoStop()
                    except Exception:
                        pass
                    was_active  = False
                    R_clutch_WC = None
                    self.status = "DISENGAGED — SPACE to re-engage"

            time.sleep(SERVO_DT)

        # Final cleanup on thread exit
        try:
            self.rtde_c.servoStop()
        except Exception:
            pass


# ============================================================================
# Main system
# ============================================================================

class VAITSystem:

    def __init__(self):
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._running = True

        # Hardware
        self.ur_robot = None
        self.stepper  = StepperMotorManager()
        self.enc_inst = None
        self.pipeline = None

        # Shared clutch state: single SPACE key controls both UR and stepper.
        # Stored as a one-element list so threads can read by reference.
        self._clutch_ref = [False]   # _clutch_ref[0] is the live bool

        # Threads / state
        self.enc_state   = EncoderFollowState()
        self.enc_thread  = None
        self.t265_thread = None
        self.listener    = None

    # ── Signal handler ───────────────────────────────────────────────────────

    def _signal_handler(self, signum, frame):
        print(f"\nSignal {signum} — shutting down...")
        self._running          = False
        self.enc_state.running = False

    # ── Initialization ───────────────────────────────────────────────────────

    def initialize(self) -> bool:
        # ── UR robot ──────────────────────────────────────────────────
        self.ur_robot = URRobotManager()
        print("Connecting to UR robot...", end=" ", flush=True)
        self.ur_robot.connect()
        if not self.ur_robot.connected:
            print("FAILED")
            return False
        print("OK")

        init_pose  = UR_CONFIG["init_pose"]
        init_speed = UR_CONFIG["init_move_speed"]
        init_accel = UR_CONFIG["init_move_accel"]
        init_rad   = [np.deg2rad(v) for v in init_pose]
        print(f"[Init] Moving to init joints (deg): {init_pose}")
        try:
            self.ur_robot.rtde_c.moveJ(init_rad, init_speed, init_accel)
            print("[Init] Reached init pose.")
        except Exception as e:
            print(f"[Init] moveJ failed: {e}")
            return False

        # ── Stepper motors ─────────────────────────────────────────────
        print("Connecting to stepper motors...", end=" ", flush=True)
        if not self.stepper.connect():
            print("FAILED")
            return False
        print("OK")

        # ── Rotary encoder ─────────────────────────────────────────────
        print("Finding encoder port...", end=" ", flush=True)
        try:
            enc_port = _find_encoder_port(
                skip_port=GRIPPER_CONFIG["stepper"]["port"]
            )
        except RuntimeError as e:
            print(f"FAILED: {e}")
            return False
        self.enc_inst = _make_enc_instrument(enc_port)
        raw = _read_angle(self.enc_inst)
        if raw is not None:
            self.enc_state.set_offset(raw)
            print(f"OK  (port={enc_port}, offset={raw:.2f}°)")
        else:
            print(f"OK  (port={enc_port}, could not read angle — offset=0°)")

        # ── T265 pipeline ─────────────────────────────────────────────
        if not RS_AVAILABLE or not SCIPY_AVAILABLE:
            print("T265 / scipy not available — T265 tracking disabled.")
            return False

        print("Starting T265 pipeline...", end=" ", flush=True)
        self.pipeline = rs.pipeline()
        rs_cfg = rs.config()
        rs_cfg.enable_stream(rs.stream.pose)
        try:
            self.pipeline.start(rs_cfg)
        except Exception as e:
            print(f"FAILED: {e}")
            return False
        print("OK")
        time.sleep(1.0)   # let T265 stream stabilise

        # ── Background threads ────────────────────────────────────────
        self.enc_thread = EncoderFollowThread(
            self.enc_inst, self.stepper, self.enc_state
        )
        self.t265_thread = T265ControlThread(
            self.ur_robot.rtde_r, self.ur_robot.rtde_c, self.pipeline,
            self._clutch_ref,
        )
        self.enc_thread.start()
        self.t265_thread.start()

        return True

    # ── Keyboard handler ─────────────────────────────────────────────────────

    def _on_press(self, key):
        try:
            ch = key.char
            if ch == 'z':
                threading.Thread(target=self._home_and_zero, daemon=True).start()
        except AttributeError:
            if key == keyboard.Key.space:
                self._toggle_clutch()
            elif key == keyboard.Key.esc:
                self._running          = False
                self.enc_state.running = False
                return False

    def _toggle_clutch(self):
        """SPACE: engage or disengage both UR arm and encoder/stepper together."""
        engaging = not self._clutch_ref[0]
        self._clutch_ref[0] = engaging

        if engaging:
            # Re-anchor encoder offset so steppers resume from current position
            raw = self.enc_thread.get_latest_raw() if self.enc_thread else None
            if raw is not None:
                saved = self.enc_thread._last_cmd_angle
                self.enc_state.set_offset(raw - saved)
                self.enc_thread.set_resume_angle(saved)
            self.enc_state.set_follow_mode(True)
        else:
            self.enc_state.set_follow_mode(False)

    def _home_and_zero(self):
        """Z key (threaded): home steppers to center then zero encoder reference."""
        self._clutch_ref[0] = False
        self.enc_state.set_follow_mode(False)
        spd = GRIPPER_CONFIG["stepper"]["homing_speed"]
        self.stepper.send_move_command(1000, 1000, spd)
        time.sleep(1.5)
        self.stepper.send_move_command(0, 0, spd)
        time.sleep(2.5)
        self.stepper.send_move_command(500, 500, spd)
        time.sleep(1.0)
        raw = self.enc_thread.get_latest_raw() if self.enc_thread else None
        if raw is not None:
            self.enc_state.set_offset(raw)
        if self.enc_thread:
            self.enc_thread.reset_tracking()

    # ── Display ──────────────────────────────────────────────────────────────

    def _update_display(self):
        print('\033[2J\033[H', end='')

        # ── Shared clutch + status ──
        clutch   = self._clutch_ref[0]
        ur_conn  = self.ur_robot.connected if self.ur_robot else False
        ur_status = self.t265_thread.status if self.t265_thread else "unavailable"
        pose      = self.t265_thread.current_pose if self.t265_thread else None
        pose_str  = (
            "xyz=[" + ", ".join(f"{v:.4f}" for v in pose[:3]) + "]"
            if pose else "—"
        )

        # ── Stepper / encoder status ──
        follow  = self.enc_state.get_follow_mode()
        raw     = self.enc_thread.get_latest_raw() if self.enc_thread else None
        rel     = (raw - self.enc_state.get_offset()) if raw is not None else None
        enc_str = (
            f"{rel:+.2f}°  (overflow {self.enc_thread._overflow:+.1f}°)"
            if rel is not None else "read error"
        )
        step_mode = "FOLLOW — encoder drives steppers" if follow else "STANDBY"

        clutch_label = "ENGAGED" if clutch else "DISENGAGED"

        print("=" * 64)
        print("                  VAIT CONTROL SYSTEM")
        print("=" * 64)
        print()
        print(f"  Clutch  : {clutch_label}")
        print()
        print("  ── UR ARM (T265 Teleop) ──────────────────────────────")
        print(f"  Robot   : {'Connected' if ur_conn else 'Disconnected'}")
        print(f"  Status  : {ur_status}")
        print(f"  TCP pos : {pose_str}")
        print()
        print("  ── GRIPPER (Encoder Follow) ──────────────────────────")
        print(f"  Mode    : {step_mode}")
        print(f"  Encoder : {enc_str}")
        print(f"  Steppers: M1={self.stepper.m1_target_pos:<5}  "
              f"M2={self.stepper.m2_target_pos:<5}  "
              f"({'Connected' if self.stepper.connected else 'Disconnected'})")
        print()
        print("-" * 64)
        print("  SPACE  engage/disengage  |  Z  Home+zero  |  ESC  Quit")
        print("=" * 64)

    # ── Run ──────────────────────────────────────────────────────────────────

    def run(self):
        try:
            if not self.initialize():
                print("Initialization failed. Exiting.")
                return

            self.listener = keyboard.Listener(on_press=self._on_press)
            self.listener.start()

            print()
            print("=" * 64)
            print("             VAIT CONTROL SYSTEM READY")
            print("=" * 64)
            print("  SPACE  — engage / disengage (UR arm + stepper together)")
            print("  Z      — Home + zero encoder    ESC — Quit")
            print("=" * 64)
            print()

            last_display = 0.0
            while self._running:
                now = time.time()
                if now - last_display >= _DISPLAY_INTERVAL:
                    self._update_display()
                    last_display = now
                time.sleep(0.05)

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def cleanup(self):
        print("\nShutting down...")
        self._running          = False
        self.enc_state.running = False

        if self.t265_thread:
            self.t265_thread.stop()
            self.t265_thread.join(timeout=2.0)

        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass

        if self.listener:
            try:
                self.listener.stop()
            except Exception:
                pass

        self.stepper.disconnect()
        if self.ur_robot:
            self.ur_robot.disconnect()

        print("Shutdown complete.")
        os._exit(0)


# ============================================================================
# Entry point
# ============================================================================

def main():
    VAITSystem().run()


if __name__ == "__main__":
    main()
