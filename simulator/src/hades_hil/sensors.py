"""6DOF state -> virtual sensor measurements.

Sign convention (must match the firmware, see estimation.cpp):
the BMI088 driver reports a_raw such that at rest, upright, a_raw_z = -g.
That is  a_raw = R_world->body @ (a_world - [0, 0, +g])  with z up, where
a_world is the net (coordinate) acceleration.  Checks:
  on the pad:  a_world = 0      -> a_raw_z = -9.81  (firmware tilt = 0)
  3g boost  :  a_world = +30 z  -> a_raw_z = +20.19 (verticalAccel() -> 30 - g + g)
"""

from dataclasses import dataclass

import numpy as np

from .config import GRAVITY, NoiseConfig


def quat_to_rotmat(e0, e1, e2, e3):
    """Body->world rotation matrix from a RocketPy quaternion [e0,e1,e2,e3]."""
    return np.array([
        [e0**2 + e1**2 - e2**2 - e3**2, 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2)],
        [2 * (e1 * e2 + e0 * e3), e0**2 - e1**2 + e2**2 - e3**2, 2 * (e2 * e3 - e0 * e1)],
        [2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), e0**2 - e1**2 - e2**2 + e3**2],
    ])


@dataclass
class SensorSample:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    hpa: float
    temp_c: float


class SensorSimulator:
    """Converts true state to noisy BMI088 + BMP585 readings."""

    def __init__(self, environment, noise: NoiseConfig):
        self.env = environment
        self.noise = noise
        rng = np.random.default_rng(noise.seed)
        self._rng = rng
        self.accel_bias = rng.normal(0.0, noise.accel_bias_std, 3)
        self.gyro_bias = rng.normal(0.0, noise.gyro_bias_std, 3)

    def pad_pressure_hpa(self) -> float:
        return float(self.env.pressure(self.env.elevation)) / 100.0

    def pad_temp_c(self) -> float:
        return float(self.env.temperature(self.env.elevation)) - 273.15

    def measure(self, z_asl, quat, omega_body, accel_world) -> SensorSample:
        rng = self._rng
        n = self.noise

        rot = quat_to_rotmat(*quat)
        specific = np.asarray(accel_world, dtype=float) - np.array([0.0, 0.0, GRAVITY])
        a_body = rot.T @ specific + self.accel_bias + rng.normal(0.0, n.accel_std, 3)

        g_body = np.asarray(omega_body, dtype=float) + self.gyro_bias \
            + rng.normal(0.0, n.gyro_std, 3)

        hpa = float(self.env.pressure(z_asl)) / 100.0 + rng.normal(0.0, n.baro_std_hpa)
        temp_c = float(self.env.temperature(z_asl)) - 273.15 + rng.normal(0.0, n.temp_std_c)

        return SensorSample(a_body[0], a_body[1], a_body[2],
                            g_body[0], g_body[1], g_body[2],
                            hpa, temp_c)
