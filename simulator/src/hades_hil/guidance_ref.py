"""Pure-Python port of the firmware guidance law (src/guidance.cpp).

Used by the offline reference run and the virtual Teensy.  Any change to the
firmware guidance must be mirrored here for the A/B comparison to stay valid.
"""

import math

from .cd_table import lookup_cd
from .config import (GRAVITY, GUIDANCE_KP, ROCKET_MASS_KG, ROCKET_REF_AREA_M2,
                     TARGET_APOGEE_M, ACT_MAX_POS_CM)


def predict_apogee(h, v, cd, rho,
                   mass=ROCKET_MASS_KG, area=ROCKET_REF_AREA_M2):
    """Energy-balance apogee with mean-drag correction (v/2 average)."""
    if v <= 0.0:
        return h
    k = (rho * cd * area) / (4.0 * mass)
    return h + (v * v) / (2.0 * GRAVITY + k * v * v)


def guidance_update(h, v, hpa, temp_c, current_opening,
                    target_apogee=TARGET_APOGEE_M, kp=GUIDANCE_KP):
    """Returns (target_opening 0..1, predicted_apogee_m, apogee_error_m)."""
    t_k = temp_c + 273.15
    p_pa = hpa * 100.0
    rho = p_pa / (287.05 * t_k)
    sound = 20.05 * math.sqrt(t_k)
    mach = v / sound if v > 0.0 and sound > 0.0 else 0.0

    cd = lookup_cd(mach, current_opening)
    apogee = predict_apogee(h, v, cd, rho)
    error = apogee - target_apogee

    opening = min(max(kp * error, 0.0), 1.0)
    return opening, apogee, error


class RateLimitedActuator:
    """First-order stand-in for the linear actuator in offline runs.

    Moves toward the commanded opening at the HIL plant's max speed
    (ACT_MAX_SPEED over the full stroke).  The real inner PID is only
    exercised in HIL runs.
    """

    def __init__(self, max_speed_cms, stroke_cm=ACT_MAX_POS_CM):
        self.opening = 0.0
        self.rate = max_speed_cms / stroke_cm   # fraction of stroke per second

    def step(self, target_opening, dt):
        delta = target_opening - self.opening
        max_step = self.rate * dt
        self.opening += min(max(delta, -max_step), max_step)
        self.opening = min(max(self.opening, 0.0), 1.0)
        return self.opening
