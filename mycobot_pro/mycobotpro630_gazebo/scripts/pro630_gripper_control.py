"""Convert the selected percentage to the gripper SDK speed parameter."""
import math

def gripper_speed_for_percentage(start, goal, speed_scale):
    """Slider percentage selects the SDK's 1..100 range, independently of arm travel."""
    if (not all(math.isfinite(v) for v in (start, goal, speed_scale))
            or not 0 <= start <= 100 or not 0 <= goal <= 100
            or not 0 < speed_scale <= 1):
        raise ValueError('invalid gripper speed percentage')
    if abs(goal - start) < 0.5:
        return None
    return max(1, min(100, int(math.floor(speed_scale * 100.0 + 0.5))))

