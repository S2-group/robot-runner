import math

def rotation_is_close(target, yaw) -> bool:
    return math.isclose(target, yaw, abs_tol=10**-2)