"""Airbrakes drag table — Python mirror of HADES-Airbrakes src/guidance.cpp.

Full-vehicle Cd from CFD at 0/25/50/75 deg flap deflection, referenced to the
body cross-section.  lookup_cd() reproduces the firmware's clamped bilinear
interpolation exactly so the simulated plant and the onboard predictor share
one aero model.
"""

MACH_AXIS = [0.25, 0.50, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00]
OPEN_AXIS = [0.0, 0.3333, 0.6667, 1.0]

CD_TABLE = [
    [0.336, 0.594, 1.012, 1.298],  # Mach 0.25
    [0.304, 0.581, 1.051, 1.352],  # Mach 0.50
    [0.289, 0.624, 1.202, 1.529],  # Mach 0.75
    [0.287, 0.647, 1.254, 1.583],  # Mach 0.80
    [0.286, 0.686, 1.318, 1.649],  # Mach 0.85
    [0.288, 0.754, 1.415, 1.740],  # Mach 0.90
    [0.338, 0.854, 1.596, 1.908],  # Mach 0.95
    [0.450, 1.013, 1.805, 2.177],  # Mach 1.00
]


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def lookup_cd(mach: float, opening: float) -> float:
    mach = _clamp(mach, MACH_AXIS[0], MACH_AXIS[-1])
    opening = _clamp(opening, OPEN_AXIS[0], OPEN_AXIS[-1])

    mi = len(MACH_AXIS) - 2
    for i in range(len(MACH_AXIS) - 1):
        if mach <= MACH_AXIS[i + 1]:
            mi = i
            break

    oi = len(OPEN_AXIS) - 2
    for i in range(len(OPEN_AXIS) - 1):
        if opening <= OPEN_AXIS[i + 1]:
            oi = i
            break

    tm = (mach - MACH_AXIS[mi]) / (MACH_AXIS[mi + 1] - MACH_AXIS[mi])
    to = (opening - OPEN_AXIS[oi]) / (OPEN_AXIS[oi + 1] - OPEN_AXIS[oi])

    c00, c01 = CD_TABLE[mi][oi], CD_TABLE[mi][oi + 1]
    c10, c11 = CD_TABLE[mi + 1][oi], CD_TABLE[mi + 1][oi + 1]

    return (c00 * (1 - tm) * (1 - to) + c10 * tm * (1 - to)
            + c01 * (1 - tm) * to + c11 * tm * to)


def rocketpy_drag_curve(deployment_level: float, mach: float) -> float:
    """Argument order expected by RocketPy AirBrakes (Deployment Level, Mach)."""
    return lookup_cd(mach, deployment_level)


def closed_brakes_drag_curve(mach_max: float = 2.0, step: float = 0.01):
    """Rocket power-off drag as (Mach, Cd) pairs: the table's 0-opening column.

    RocketPy only evaluates the airbrakes curve when deployment_level > 0 and
    otherwise falls back to the rocket's own power_off_drag.  Using the same
    column here keeps Cd continuous at deployment 0.
    """
    n = int(round(mach_max / step))
    return [(i * step, lookup_cd(i * step, 0.0)) for i in range(n + 1)]
