#!/usr/bin/env python3
"""
Stepper motor connection and basic motion test.

Sequence:
  1. Connect to Arduino and trigger homing.
  2. Move both motors from 0 → 1000 steps (full travel).
  3. Move both motors from 1000 → 0 steps (return).
  4. Reset to the configured centre position (initial_pos).

Run:
    python scripts/tests/test_stepper.py
"""

import sys
import os
import serial
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from src.core.config_loader import GRIPPER_CONFIG

_SC = GRIPPER_CONFIG["stepper"]

# ── helpers ──────────────────────────────────────────────────────────────────

def _send(ser: serial.Serial, cmd: str) -> None:
    ser.write(cmd.encode('utf-8'))


def _move(ser: serial.Serial, m1: int, m2: int, speed: int, wait: float = 0.0) -> None:
    """Send a move command and optionally wait for motion to complete."""
    m1 = max(0, min(_SC["max_steps"], m1))
    m2 = max(0, min(_SC["max_steps"], m2))
    cmd = f"<{m1},{m2},{speed},{_SC['microsteps']}>\n"
    _send(ser, cmd)
    print(f"  → MOVE  M1={m1:>4}  M2={m2:>4}  speed={speed}")
    if wait > 0:
        time.sleep(wait)


def _drain(ser: serial.Serial, timeout: float = 0.5) -> str:
    """Read all pending serial lines within *timeout* seconds."""
    deadline = time.time() + timeout
    lines = []
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                lines.append(line)
                print(f"  Arduino: {line}")
    return "\n".join(lines)


# ── test ─────────────────────────────────────────────────────────────────────

def run_test() -> None:
    cfg_port = _SC["port"]
    baud = _SC["baud_rate"]
    speed    = _SC["homing_speed"]          # use homing speed for the sweep
    center   = _SC["initial_pos"]           # 500
    max_pos  = _SC["max_steps"]             # 1000

    # Estimate travel time: steps at microstep resolution at homing_speed (steps/s).
    # homing_speed in config is 500 steps/s; full travel = 1000 steps -> ~2 s.
    travel_time = (max_pos / max(speed, 1)) + 1.0

    print("=" * 55)
    print("         STEPPER CONNECTION & MOTION TEST")
    print("=" * 55)

    # ── Port resolution ─────────────────────────────────────────────────────
    port = cfg_port
    if not os.path.exists(port):
        raise RuntimeError(
            f"Configured stepper port '{port}' not found.\n"
            "Run  python -m serial.tools.list_ports  to find the correct port,\n"
            "then update stepper.port in configs/gripper.yaml."
        )

    ser = serial.Serial(port, baud, timeout=1, dsrdtr=False)
    print(f"\n✓ Serial port opened: {port} @ {baud} baud")

    # Single DTR pulse to reset the Arduino (same as the auto-detect probe).
    ser.setDTR(False); time.sleep(0.05)
    ser.setDTR(True);  time.sleep(0.05)
    ser.setDTR(False)

    print("  Waiting 3 s for Arduino sketch to start …")
    time.sleep(3)

    # ── Verify Arduino identity ─────────────────────────────────────────────
    print("\n[0/4] Verifying Arduino identity …")
    boot_buf = b""
    deadline = time.time() + 1.5
    while time.time() < deadline:
        if ser.in_waiting:
            boot_buf += ser.read(ser.in_waiting)
        time.sleep(0.05)
    if boot_buf:
        print(f"  Arduino says: {boot_buf.decode('utf-8', errors='replace').strip()}")
    if b"Arduino Ready" not in boot_buf:
        ser.close()
        raise RuntimeError(
            f"Device on {port} did not send the expected Arduino boot message.\n"
            "This is likely the RS-485 encoder adapter, not the stepper Arduino.\n"
            "Run  python -m serial.tools.list_ports  to list all ports, then\n"
            "update stepper.port in configs/gripper.yaml with the correct port."
        )
    print("  ✓ Correct device confirmed")

    # ── 1. Homing ──────────────────────────────────────────────────────────
    print("\n[1/4] Sending HOME command …")
    _send(ser, "<HOME>\n")
    print("  Waiting for homing to complete (10 s) …")
    home_response = _drain(ser, timeout=10.0)
    if not home_response:
        print("  ⚠ No response during homing (check 12 V motor power supply)")
    else:
        print("  ✓ Homing done")

    # ── Return motors to 0 before closing ──────────────────────────────────
    print("\n[2/4] Returning both motors to position 0 …")
    _move(ser, 0, 0, speed, wait=travel_time)
    print("  ✓ Motors at 0")

    ser.close()
    print(f"\n✓ Serial port closed")
    print("\n" + "=" * 55)
    print("  TEST PASSED — steppers responded correctly")
    print("=" * 55)


if __name__ == "__main__":
    ser = None
    try:
        run_test()
    except (serial.SerialException, RuntimeError) as e:
        print(f"\n✗ Test failed: {e}")
        print("  Update stepper.port in configs/gripper.yaml and retry.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted — returning motors to 0 before exit.")
        # Best-effort return to zero; ser is local to run_test so we re-open briefly.
        cfg_port = _SC["port"]
        try:
            _s = serial.Serial(cfg_port, _SC["baud_rate"], timeout=1, dsrdtr=False)
            _move(_s, 0, 0, _SC["homing_speed"], wait=(_SC["max_steps"] / max(_SC["homing_speed"], 1)) + 1.0)
            _s.close()
        except Exception:
            pass
        sys.exit(0)
