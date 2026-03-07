import yaml
import os

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'system_config.yaml')

with open(_CONFIG_PATH, 'r') as f:
    _data = yaml.safe_load(f)

CONFIG = _data.get('CONFIG', {})
GRIPPER_MIN_POS = _data.get('GRIPPER_MIN_POS', -1.37)
GRIPPER_MAX_POS = _data.get('GRIPPER_MAX_POS', 0.0)
ADAPTIVE_GRIPPING_CONFIG = _data.get('ADAPTIVE_GRIPPING_CONFIG', {})
DISPLAY_CONFIG = _data.get('DISPLAY_CONFIG', {})
CENTERLINE_CONFIG = _data.get('CENTERLINE_CONFIG', {})
RECORDING_CONFIG = _data.get('RECORDING_CONFIG', {})
SAFETY_CONFIG = _data.get('SAFETY_CONFIG', {})
