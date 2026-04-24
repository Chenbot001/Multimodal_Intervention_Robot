import yaml
import os

_CFG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'configs')


def _load(filename: str) -> dict:
    with open(os.path.join(_CFG_DIR, filename), 'r') as f:
        return yaml.safe_load(f)


# ── Per-device configs ────────────────────────────────────────────────────────
GRIPPER_CONFIG = _load('gripper.yaml')    # end-effector: steppers + DM motor + gripping
UR_CONFIG      = _load('ur_robot.yaml')   # UR arm + T265 teleop parameters
SENSOR_CONFIG  = _load('sensors.yaml')    # Daimon visuotactile + Bota F/T + safety thresholds

# ── Application-level config ──────────────────────────────────────────────────
SYSTEM_CONFIG  = _load('system.yaml')

# ── Convenience aliases (sub-dicts used directly throughout the codebase) ─────
ADAPTIVE_GRIPPING_CONFIG = GRIPPER_CONFIG['adaptive_gripping']
CENTERLINE_CONFIG        = SENSOR_CONFIG['daimon']['centerline']
SAFETY_CONFIG            = SENSOR_CONFIG['safety']
DISPLAY_CONFIG           = SYSTEM_CONFIG['display']
RECORDING_CONFIG         = SYSTEM_CONFIG['recording']

# ── Scalar aliases kept for utils.py and any other direct users ───────────────
GRIPPER_MIN_POS = GRIPPER_CONFIG['motor']['min_pos']
GRIPPER_MAX_POS = GRIPPER_CONFIG['motor']['max_pos']
