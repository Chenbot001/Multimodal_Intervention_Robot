"""
Utility functions for the Gripper Control System
Contains helper functions for gripper position conversion
"""


def percentage_to_position(percentage):
    """Convert closure percentage to radian position"""
    from src.core.config_loader import GRIPPER_MAX_POS, GRIPPER_MIN_POS
    
    percentage = max(0, min(100, percentage))
    position = GRIPPER_MAX_POS + (percentage/100.0) * (GRIPPER_MIN_POS - GRIPPER_MAX_POS)
    return position


def position_to_percentage(position):
    """Convert radian position to closure percentage"""
    from src.core.config_loader import GRIPPER_MAX_POS, GRIPPER_MIN_POS
    
    position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    percentage = ((position - GRIPPER_MAX_POS) / (GRIPPER_MIN_POS - GRIPPER_MAX_POS)) * 100
    return percentage


def safe_gripper_position(position):
    """Ensures the gripper position stays within safe limits"""
    from src.core.config_loader import GRIPPER_MIN_POS, GRIPPER_MAX_POS
    
    limited_position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    return limited_position
