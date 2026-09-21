"""Bridges between RocketPy's airbrakes controller callback and either the
real Teensy (HilBridge) or the Python guidance port (OfflineBridge).

RocketPy state vector: [x, y, z, vx, vy, vz, e0, e1, e2, e3, w1, w2, w3]
z is ASL; the firmware altitude is AGL (baro-relative), so truth comparisons
subtract the launch site elevation.
"""

import time

import numpy as np

from . import protocol
from .config import (ACT_MAX_POS_CM, ACT_MAX_SPEED_CMS, BARO_EVERY_N,
                     IMU_DT_US, IMU_RATE_HZ)
from .guidance_ref import RateLimitedActuator, guidance_update
from .logger import FlightLog
from .sensors import SensorSimulator


class HilBridge:
    """RocketPy controller that closes the loop through the Teensy.

    Lock-step: each controller call sends one SENSOR packet (one IMU sample
    of simulated time) and blocks for the STATUS reply.  The reported
    actuator position becomes the RocketPy deployment level, so the
    simulated trajectory feels the real control loop's latency, actuator
    rate limits and estimation error.

    Before the first flight packet, `settle_s` seconds of static pad data
    are streamed so the Madgwick and Kalman filters converge — mirrors the
    pad wait before launch.

    With `real_time=True` each packet is held until the wall clock reaches
    its sim timestamp, so 1 s of simulation takes 1 s of real time.  A
    physical actuator (teensy41-hil-act) moves in wall time, so without
    pacing its speed relative to the flight depends on how fast the host
    happens to run.
    """

    def __init__(self, link, sensor_sim: SensorSimulator, settle_s: float = 8.0,
                 real_time: bool = True, lag_warn_s: float = 0.05):
        self.link = link
        self.sensors = sensor_sim
        self.settle_s = settle_s
        self.real_time = real_time
        self.lag_warn_s = lag_warn_s
        self.log = FlightLog()
        self.last_status = None
        self.rt_lag_s = 0.0            # lag of the most recent packet
        self.rt_max_lag_s = 0.0
        self._packet_idx = 0
        self._settle_packets = 0
        self._settled = False
        self._prev_t = None
        self._prev_vel = np.zeros(3)
        self._wall0 = None             # wall time of sim t=0 (perf_counter)
        self._last_lag_warn = 0.0

    # --- helpers ----------------------------------------------------------

    def _pace(self, sim_time_us: int):
        """Block until the wall clock catches up with `sim_time_us`.

        Paced against an absolute anchor (`_wall0`) so per-packet sleep
        jitter cannot accumulate into clock drift.  Coarse-sleeps to ~1 ms
        before the target, then spins, since time.sleep() alone can
        overshoot by more than the 5 ms packet period.
        """
        now = time.perf_counter()
        if self._wall0 is None:
            self._wall0 = now - sim_time_us * 1e-6   # first packet is on time
            self.rt_lag_s = 0.0
            return
        target = self._wall0 + sim_time_us * 1e-6
        remaining = target - now
        self.rt_lag_s = max(0.0, -remaining)
        if remaining > 0:
            if remaining > 0.002:
                time.sleep(remaining - 0.001)
            while time.perf_counter() < target:
                pass
        else:
            self.rt_max_lag_s = max(self.rt_max_lag_s, self.rt_lag_s)
            if (self.rt_lag_s >= self.lag_warn_s
                    and now - self._last_lag_warn >= 1.0):
                print(f"[hil] warning: sim {self.rt_lag_s * 1e3:.0f} ms behind "
                      "real time (serial round-trip + integration slower than "
                      "the packet period)")
                self._last_lag_warn = now

    def _flags(self) -> int:
        flags = protocol.FLAG_IMU
        if self._packet_idx % BARO_EVERY_N == 0:
            flags |= protocol.FLAG_BARO
        return flags

    def _send(self, sim_time_us, z_asl, quat, omega, accel_world) -> protocol.Status:
        if self.real_time:
            self._pace(sim_time_us)
        sample = self.sensors.measure(z_asl, quat, omega, accel_world)
        frame = protocol.make_sensor(
            sim_time_us,
            sample.ax, sample.ay, sample.az,
            sample.gx, sample.gy, sample.gz,
            sample.hpa, sample.temp_c,
            flags=self._flags(),
        )
        self._packet_idx += 1
        status = self.link.exchange(frame)
        self.last_status = status
        return status

    def _pad_settle(self, z_asl, quat):
        n = int(self.settle_s * IMU_RATE_HZ)
        pace = "paced to real time" if self.real_time else "as fast as possible"
        print(f"[hil] pad settling: {n} static packets "
              f"({self.settle_s:.0f} s sim time, {pace})")
        status = None
        for i in range(1, n + 1):
            status = self._send(i * IMU_DT_US, z_asl, quat,
                                np.zeros(3), np.zeros(3))
        self._settle_packets = n
        if status is not None:
            print(f"[hil] settled: phase={status.phase_name} "
                  f"estH={status.est_h:.2f} m  tilt={status.tilt_deg:.1f} deg "
                  f"actPos={status.actual_pos_cm:.2f} cm")

    # --- RocketPy callback --------------------------------------------------

    def controller(self, time, sampling_rate, state, state_history,
                   observed_variables, air_brakes, *args):
        z_asl = state[2]
        vel = np.array(state[3:6])
        quat = state[6:10]
        omega = state[10:13]

        if not self._settled:
            self._pad_settle(z_asl, quat)
            self._settled = True
            self._prev_t = 0.0
            self._prev_vel = np.zeros(3)

        # World-frame acceleration by finite difference of velocity between
        # controller calls (dt = 1/sampling_rate of smooth integrator output).
        if time > self._prev_t:
            accel_world = (vel - self._prev_vel) / (time - self._prev_t)
        else:
            accel_world = np.zeros(3)
        self._prev_t = time
        self._prev_vel = vel.copy()

        sim_time_us = self._settle_packets * IMU_DT_US + int(round(time * 1e6))
        status = self._send(sim_time_us, z_asl, quat, omega, accel_world)

        deployment = min(max(status.actual_pos_cm / ACT_MAX_POS_CM, 0.0), 1.0)
        air_brakes.deployment_level = deployment

        env = self.sensors.env
        z_agl = z_asl - env.elevation
        speed = float(np.linalg.norm(vel))
        mach = speed / float(env.speed_of_sound(z_asl))

        self.log.add(
            t=time, sim_time_us=sim_time_us,
            z_agl=z_agl, vz=vel[2], speed=speed, mach=mach,
            deployment=deployment,
            phase=status.phase,
            est_h=status.est_h, est_v=status.est_v, tilt_deg=status.tilt_deg,
            est_apogee_m=status.est_apogee_m, apogee_err_m=status.apogee_err_m,
            target_pos_cm=status.target_pos_cm, actual_pos_cm=status.actual_pos_cm,
            kf_p00=status.kf_p00, kf_p11=status.kf_p11,
            act_duty_pct=status.act_duty_pct,
            rt_lag_ms=self.rt_lag_s * 1e3 if self.real_time else float("nan"),
        )
        return (time, deployment, status.est_apogee_m)


class OfflineBridge:
    """Same control law, no hardware: Python port of guidance + a
    rate-limited actuator.  Reference for validating HIL results and for
    running the sim without the Teensy.
    """

    # Firmware gates deployment on COASTING: launch detect + SM_COAST_DELAY_US.
    COAST_START_S = 4.45

    def __init__(self, environment, controlled: bool = True):
        self.env = environment
        self.controlled = controlled
        self.actuator = RateLimitedActuator(ACT_MAX_SPEED_CMS)
        self.log = FlightLog()
        self._prev_t = None

    def controller(self, time, sampling_rate, state, state_history,
                   observed_variables, air_brakes, *args):
        z_asl = state[2]
        vz = state[5]
        vel = np.array(state[3:6])
        z_agl = z_asl - self.env.elevation

        dt = (time - self._prev_t) if self._prev_t is not None else 1.0 / sampling_rate
        self._prev_t = time

        hpa = float(self.env.pressure(z_asl)) / 100.0
        temp_c = float(self.env.temperature(z_asl)) - 273.15

        target_opening, est_apogee, apogee_err = guidance_update(
            z_agl, vz, hpa, temp_c, self.actuator.opening)

        if not self.controlled or time < self.COAST_START_S or vz <= 0.0:
            target_opening = 0.0

        deployment = self.actuator.step(target_opening, dt) if dt > 0 else self.actuator.opening
        air_brakes.deployment_level = deployment

        speed = float(np.linalg.norm(vel))
        mach = speed / float(self.env.speed_of_sound(z_asl))
        self.log.add(
            t=time, z_agl=z_agl, vz=vz, speed=speed, mach=mach,
            deployment=deployment,
            est_apogee_m=est_apogee, apogee_err_m=apogee_err,
            target_pos_cm=target_opening * ACT_MAX_POS_CM,
            actual_pos_cm=deployment * ACT_MAX_POS_CM,
        )
        return (time, deployment, est_apogee)
