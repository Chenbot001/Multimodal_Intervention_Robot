"""
Stepper_Control_Encoder.py
Encoder-driven 1:1 rotation control for the VAIT gripper — stepper motors only.

The operator rotates a physical input knob connected to a Modbus absolute
rotary encoder.  A background thread reads the encoder angle at 20 Hz and
servo-commands the two finger-mounted stepper motors to track it in
real-time via differential drive:

    M1 = CENTER + angle_deg × STEPS_PER_DEG
    M2 = CENTER - angle_deg × STEPS_PER_DEG

The encoder angle is expressed relative to a software zero that can be
reset at any time without touching the encoder's hardware register.

Controls
--------
  Z         – Home steppers (→1000 → 0 → 500) then zero encoder to 0°
  SPACE     – Clutch: pause / resume follow while preserving stepper position
  ESC       – Quit
"""

import os
import sys
import time
import signal
import threading

import serial
import minimalmodbus
from pynput import keyboard

# ---------------------------------------------------------------------------
# Project path setup
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.core.config_loader import GRIPPER_CONFIG, DISPLAY_CONFIG
from src.core.hardware_manager import StepperMotorManager, find_stepper_port

# ---------------------------------------------------------------------------
# Encoder hardware constants  (mirror read_angle.py)
# ---------------------------------------------------------------------------
_SLAVE_ADDRESS   = 1
_BAUDRATE        = 9600
_READ_REG        = 0           # 32-bit multi-turn position register
_SINGLE_TURN_RES = 2 ** 15     # encoder counts per revolution

# ---------------------------------------------------------------------------
# Mapping calibration (tunable at the top of this file)
# ---------------------------------------------------------------------------
# Stepper travel: 0–1000 steps, center = 500  →  ±500 differential units.
# Map ±720° of encoder input to ±500 stepper units (two full turns = full range).
STEPS_PER_DEG: float = 500.0 / 720.0   # ≈ 0.69 steps per degree

# Ignore encoder changes smaller than this to suppress noise jitter.
# Run test_encoder_noise.py to calibrate this value for your hardware.
ANGLE_DEADBAND_DEG: float = 1.0

# Encoder read rate
ENCODER_POLL_HZ: int = 10

# ---------------------------------------------------------------------------
# Encoder helpers
# ---------------------------------------------------------------------------

def find_encoder_port() -> str:
    """
    Return the encoder port from config (auto-detection disabled).
    Update configs/gripper.yaml encoder.port if the port has changed.
    """
    port = GRIPPER_CONFIG["encoder"]["port"]
    print(f"Using encoder port from config: {port}")
    return port


def _make_instrument(port: str) -> minimalmodbus.Instrument:
    inst = minimalmodbus.Instrument(port, _SLAVE_ADDRESS)
    inst.serial.baudrate = _BAUDRATE
    inst.serial.bytesize = 8
    inst.serial.parity   = minimalmodbus.serial.PARITY_NONE
    inst.serial.stopbits = 1
    inst.serial.timeout  = 0.5
    inst.mode = minimalmodbus.MODE_RTU
    return inst


def read_continuous_angle(instrument: minimalmodbus.Instrument):
    """
    Return the continuous multi-turn angle in degrees, or None on error.

    The encoder stores a 32-bit value:
      bits[14:0]  – single-turn position (0 … 32767)
      bits[31:15] – signed turn count
    Continuous angle = turn_count * 360 + single_turn_fraction * 360
    """
    try:
        raw         = instrument.read_long(_READ_REG, 3, False)
        single_turn = raw & 0x7FFF
        turn_count  = raw >> 15
        return turn_count * 360.0 + (single_turn / _SINGLE_TURN_RES) * 360.0
    except minimalmodbus.ModbusException:
        return None


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class EncoderFollowState:
    """Minimal thread-safe state shared between the keyboard and encoder threads."""

    def __init__(self):
        self.running             = True
        self._follow_mode        = False
        self._encoder_offset_deg = 0.0
        self._lock               = threading.Lock()

    def get_follow_mode(self) -> bool:
        with self._lock:
            return self._follow_mode

    def set_follow_mode(self, value: bool):
        with self._lock:
            self._follow_mode = value

    def get_offset(self) -> float:
        with self._lock:
            return self._encoder_offset_deg

    def set_offset(self, value: float):
        with self._lock:
            self._encoder_offset_deg = value


# ---------------------------------------------------------------------------
# Encoder follow thread
# ---------------------------------------------------------------------------

class EncoderFollowThread(threading.Thread):
    """
    Runs at ENCODER_POLL_HZ.  When follow_mode is active it reads the encoder,
    computes differential stepper targets, and calls send_move_command.
    """

    def __init__(
        self,
        instrument: minimalmodbus.Instrument,
        stepper:    StepperMotorManager,
        state:      EncoderFollowState,
    ):
        super().__init__(daemon=True, name="EncoderFollow")
        self.instrument = instrument
        self.stepper    = stepper
        self.state      = state
        self._last_commanded_angle = 0.0
        self._overflow_deg = 0.0             # absorbs encoder travel past stepper limits
        self._latest_raw  = None             # cached encoder reading (float or None)
        self._cache_lock  = threading.Lock()

    def run(self):
        dt = 1.0 / ENCODER_POLL_HZ
        while self.state.running:
            # Always read the encoder — sole owner of the hardware so there
            # is no concurrent access with the display loop (race-condition fix).
            raw = read_continuous_angle(self.instrument)
            with self._cache_lock:
                self._latest_raw = raw

            if self.state.get_follow_mode() and self.stepper.connected and raw is not None:
                relative  = raw - self.state.get_offset()
                effective = self._apply_overflow(relative)
                if abs(effective - self._last_commanded_angle) >= ANGLE_DEADBAND_DEG:
                    self._command_steppers(effective)
                    self._last_commanded_angle = effective
            time.sleep(dt)

    def _apply_overflow(self, relative: float) -> float:
        """
        Map the raw relative encoder angle to an effective angle that stays
        within the stepper travel limits.  Any rotation past a boundary is
        accumulated in _overflow_deg so that reversing direction immediately
        produces stepper movement instead of a silent dead-zone equal to the
        overshoot distance.
        """
        center    = GRIPPER_CONFIG["stepper"]["initial_pos"]
        max_s     = GRIPPER_CONFIG["stepper"]["max_steps"]
        angle_max =  (max_s - center) / STEPS_PER_DEG
        angle_min = -(center)         / STEPS_PER_DEG

        effective = relative - self._overflow_deg
        if effective > angle_max:
            self._overflow_deg += effective - angle_max
            effective = angle_max
        elif effective < angle_min:
            self._overflow_deg += effective - angle_min
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
        """Call after re-zeroing so no spurious large move is issued."""
        self._last_commanded_angle = 0.0
        self._overflow_deg         = 0.0

    def set_resume_angle(self, angle: float):
        """Prime the reference angle before re-enabling follow mode (race-free)."""
        self._last_commanded_angle = angle
        self._overflow_deg         = 0.0

    def get_latest_raw(self):
        """Return the most recent encoder reading cached by the run loop."""
        with self._cache_lock:
            return self._latest_raw


# ---------------------------------------------------------------------------
# Main control system
# ---------------------------------------------------------------------------

class StepperEncoderControlSystem:

    def __init__(self):
        self.state         = EncoderFollowState()
        self.stepper       = StepperMotorManager()
        self.encoder       = None
        self.follow_thread = None
        self.listener      = None

        # Clutch state
        self._clutch_saved_angle_deg: float = 0.0
        self._clutch_engaged: bool          = False

        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _signal_handler(self, signum, frame):
        print(f"\nSignal {signum} received — shutting down...")
        self.state.running = False

    def initialize(self) -> bool:
        print("Initializing Stepper Encoder Control System...")

        # Stepper motors
        if not self.stepper.connect():
            print("❌  Stepper connection failed.")
            return False

        # Encoder
        try:
            port = find_encoder_port()
        except RuntimeError as e:
            print(f"❌  {e}")
            return False

        self.encoder = _make_instrument(port)

        # Bootstrap offset so the knob's current position = 0°
        raw = read_continuous_angle(self.encoder)
        if raw is not None:
            self.state.set_offset(raw)
            print(f"✓ Encoder offset set to {raw:.2f}° (relative angle = 0°)")
        else:
            print("⚠  Could not read encoder on startup — offset left at 0°")

        # Start background follow thread (idle until E is pressed)
        self.follow_thread = EncoderFollowThread(self.encoder, self.stepper, self.state)
        self.follow_thread.start()
        print("✓ Encoder follow thread started")

        print("\n" + "=" * 60)
        print("       STEPPER ENCODER CONTROL SYSTEM READY")
        print("=" * 60)
        print("Press [E] to enable encoder-follow mode.")
        print()
        return True

    def cleanup(self):
        print("\nShutting down...")
        self.state.running = False
        if self.listener:
            try:
                self.listener.stop()
            except Exception:
                pass
        self.stepper.disconnect()
        print("✓ Cleanup complete")
        os._exit(0)

    # ------------------------------------------------------------------
    # Keyboard handler
    # ------------------------------------------------------------------

    def _on_press(self, key):
        try:
            ch = key.char

            # --- Z: home steppers (→1000 → 0 → 500) then zero encoder ---
            if ch == 'z':
                def _home_and_zero():
                    self.state.set_follow_mode(False)
                    self._clutch_engaged = False
                    spd = GRIPPER_CONFIG["stepper"]["homing_speed"]
                    print("\n🏠  Homing and zeroing…")
                    self.stepper.send_move_command(1000, 1000, spd)
                    time.sleep(1.5)
                    self.stepper.send_move_command(0, 0, spd)
                    time.sleep(2.5)
                    self.stepper.send_move_command(500, 500, spd)
                    time.sleep(1.0)
                    raw = self.follow_thread.get_latest_raw()
                    if raw is not None:
                        self.state.set_offset(raw)
                    self.follow_thread.reset_tracking()
                    self._clutch_saved_angle_deg = 0.0
                    print("✅  Homed — steppers at center, encoder at 0°")
                threading.Thread(target=_home_and_zero, daemon=True).start()

        except AttributeError:
            # Special keys
            if key == keyboard.Key.space:
                if self.state.get_follow_mode():
                    # ── Engage clutch: pause follow, save current stepper angle ──
                    saved = self.follow_thread._last_commanded_angle
                    self._clutch_saved_angle_deg = saved
                    self._clutch_engaged         = True
                    self.state.set_follow_mode(False)
                    print(f"\n⏸  Clutch ENGAGED — paused at {saved:+.2f}°")
                else:
                    # ── Release clutch: re-enable follow from the saved angle ──
                    enc_now = self.follow_thread.get_latest_raw()
                    if enc_now is not None:
                        new_offset = enc_now - self._clutch_saved_angle_deg
                        self.state.set_offset(new_offset)
                        self.follow_thread.set_resume_angle(self._clutch_saved_angle_deg)
                        self._clutch_engaged = False
                        self.state.set_follow_mode(True)
                        print(f"\n▶  Clutch RELEASED — resumed at {self._clutch_saved_angle_deg:+.2f}°")
                    else:
                        print("\n⚠  Cannot release clutch: encoder not ready yet")

            elif key == keyboard.Key.esc:
                self.state.running = False
                return False

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def _update_display(self):
        print('\033[2J\033[H', end='')

        mode = self.state.get_follow_mode()

        raw     = self.follow_thread.get_latest_raw()
        rel     = (raw - self.state.get_offset()) if raw is not None else None
        overflow = self.follow_thread._overflow_deg
        if rel is not None:
            enc_str = f"{rel:+.2f}°  (overflow {overflow:+.1f}°)"
        else:
            enc_str = "read error"

        print("=" * 62)
        print("        STEPPER ENCODER CONTROL SYSTEM")
        print("=" * 62)

        if mode:
            mode_label = "🟢 FOLLOW  (encoder drives steppers)"
        elif self._clutch_engaged:
            mode_label = f"⏸ CLUTCHED — saved at {self._clutch_saved_angle_deg:+.2f}°"
        else:
            mode_label = "⚪ STANDBY (encoder not tracking)"
        print(f"  Mode   : {mode_label}")
        print(f"  Encoder: {enc_str}")
        print(f"  Scale  : {STEPS_PER_DEG:.3f} steps/deg  |  "
              f"deadband {ANGLE_DEADBAND_DEG}°  |  {ENCODER_POLL_HZ} Hz")

        print("-" * 62)
        print("ROTATION — Stepper Motors:")
        print(f"  M1 = {self.stepper.m1_target_pos:<5}  "
              f"M2 = {self.stepper.m2_target_pos:<5}  "
              f"({'Connected' if self.stepper.connected else 'Disconnected'})")
        print(f"  {self.stepper.last_message}")

        print("-" * 62)
        print("CONTROLS:")
        print("  [Z] Home + zero (steppers \u21921000 \u21920 \u2192500, encoder \u21920\u00b0)")
        print("  [SPACE] Clutch toggle (pause / resume follow)  [ESC] Quit")
        print("=" * 62)

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    def run(self):
        try:
            if not self.initialize():
                print("❌  Initialization failed.")
                return

            self.listener = keyboard.Listener(on_press=self._on_press)
            self.listener.start()

            last_display = 0.0
            while self.state.running:
                now = time.time()
                if now - last_display >= DISPLAY_CONFIG["update_interval_seconds"]:
                    self._update_display()
                    last_display = now
                time.sleep(0.05)

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    StepperEncoderControlSystem().run()


if __name__ == "__main__":
    main()
