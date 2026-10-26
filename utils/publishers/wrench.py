"""Rotate force and torque components without changing the moment reference point."""

import numpy as np


def rotate_wrench(rotation, wrench):
    """Express [Fx, Fy, Fz, Tx, Ty, Tz] in axes given by a 3x3 rotation.

    This changes the component axes only. Torque remains about the ATI sensor
    origin: no translation or r x F term is applied. Force (N) and torque
    (N-mm) retain the units supplied by the ATI calibration.
    """
    rotation = np.asarray(rotation, dtype=np.float64)
    wrench = np.asarray(wrench, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    if wrench.shape != (6,):
        raise ValueError("wrench must contain [Fx, Fy, Fz, Tx, Ty, Tz]")
    return np.concatenate((rotation @ wrench[:3], rotation @ wrench[3:]))
