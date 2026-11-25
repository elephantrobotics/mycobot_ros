"""Qt-only conversions; ROS commands and feedback remain in model coordinates."""
import math

from pro630_model import HARDWARE_OFFSETS_DEG


def joint_display_offset(index, *, radians=False):
    # The optional seventh entry is normalized gripper opening, not an angle.
    offset = HARDWARE_OFFSETS_DEG[index] if index < 6 else 0.0
    return math.radians(offset) if radians else offset


def model_to_hardware(value, index, *, radians=False):
    return value - joint_display_offset(index, radians=radians)


def hardware_to_model(value, index, *, radians=False):
    return value + joint_display_offset(index, radians=radians)


def display_limits(limits, index, decimals, *, radians=False):
    scale = 10 ** decimals
    lower, upper = (model_to_hardware(value, index, radians=radians) for value in limits)
    return math.ceil(lower * scale) / scale, math.floor(upper * scale) / scale
