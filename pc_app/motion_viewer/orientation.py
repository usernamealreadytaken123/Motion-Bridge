"""Android quaternion (w,x,y,z): device axes -> ENU world, right-handed rotations.

Device +X is right, +Y is the top edge, +Z comes out of the screen in natural
orientation. World is East / magnetic North / Up. No planar-odometry remapping.
"""

import math


def rotation_matrix(quaternion):
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("Quaternion must contain four finite values")
    length = math.hypot(*values)
    if length < 1e-12:
        raise ValueError("Zero quaternion")
    w, x, y, z = (value / length for value in values)
    return (
        (1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)),
        (2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)),
        (2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)),
    )


def relative_matrix(quaternion, reference=None):
    current = rotation_matrix(quaternion)
    if reference is None:
        return current
    base = rotation_matrix(reference)
    # R_reference.T @ R_current, not the opposite multiplication order.
    return tuple(tuple(sum(base[k][i] * current[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))
