"""Shared base pose functions used by the rendered-sensor capture pipeline."""
import numpy as np


def rotation(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.stack([
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ], axis=-1).reshape(np.shape(r) + (3, 3))


def state(t):
    t = np.asarray(t, dtype=float)
    u = np.clip((t - 5.0) / 8.0, 0.0, 1.0)
    w = np.sin(np.pi * u) ** 2
    x = -20.0 + 0.50 * (1.0 - np.cos(2.0 * np.pi * u)) * w
    y = 0.35 * np.sin(2.0 * np.pi * u) * w
    z = 1.55 + 0.20 * np.sin(4.0 * np.pi * u) * w
    r = 0.08 * np.sin(2.0 * np.pi * u) * w
    p = 0.10 * np.sin(4.0 * np.pi * u) * w
    a = 0.15 * np.sin(2.0 * np.pi * u) * w
    f = np.maximum(t - 13.0, 0.0)
    ramp = 3.0
    v = 0.70
    s = np.clip(f / ramp, 0.0, 1.0)
    dx = v * ramp * (s ** 3 - 0.5 * s ** 4) + v * np.maximum(f - ramp, 0.0)
    x = x + dx
    return np.stack([x, y, z], axis=-1), rotation(r, p, a)


def quat(matrix):
    """Quaternion in xyzw order for poses near the identity orientation."""
    qw = np.sqrt(np.maximum(1.0 + np.trace(matrix), 0.0)) / 2.0
    return [
        (matrix[2, 1] - matrix[1, 2]) / (4.0 * qw),
        (matrix[0, 2] - matrix[2, 0]) / (4.0 * qw),
        (matrix[1, 0] - matrix[0, 1]) / (4.0 * qw),
        qw,
    ]
