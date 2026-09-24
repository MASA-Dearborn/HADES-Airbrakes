#!/usr/bin/env python3
"""Virtual Teensy: emulates the teensy41-hil firmware over a pseudo-terminal
so the full HIL pipeline (protocol, link, bridge, RocketPy loop) can be tested
without hardware.

    Terminal 1:  python scripts/virtual_teensy.py
                 -> prints the pty path, e.g. /dev/ttys012
    Terminal 2:  python scripts/run_hil.py --port /dev/ttys012

It runs a deliberately simple flight stack: baro altitude + alpha-beta
velocity filter, time-based phase machine, the Python guidance port, and the
same rate-limited actuator plant as the firmware HIL model.  It is a protocol
test double, not a firmware emulator.
"""

import os
import struct
import sys
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hades_hil import protocol
from hades_hil.config import (ACT_MAX_POS_CM, ACT_MAX_SPEED_CMS, GRAVITY,
                              GUIDANCE_KP, TARGET_APOGEE_M)
from hades_hil.guidance_ref import RateLimitedActuator, guidance_update

PHASES = {name: i for i, name in enumerate(protocol.PHASE_NAMES)}

LAUNCH_ACCEL = 30.0          # SM_LAUNCH_ACCEL_THRESHOLD_MS2
LAUNCH_CONFIRM = 5           # SM_LAUNCH_CONFIRM_COUNT
COAST_DELAY_US = 4_400_000   # SM_COAST_DELAY_US


class VirtualTeensy:
    def __init__(self):
        self.base_hpa = 1013.25
        self.phase = PHASES["IDLE"]
        self.launch_time_us = 0
        self.confirm = 0
        self.h = 0.0
        self.v = 0.0
        self.last_t_us = None
        self.last_hpa = None
        self.actuator = RateLimitedActuator(ACT_MAX_SPEED_CMS)
        self.target_opening = 0.0
        self.est_apogee = 0.0
        self.apogee_err = 0.0
        self.tilt = 0.0

    def on_sensor(self, p):
        (t_us, ax, ay, az, gx, gy, gz, hpa, temp_c, mx, my, mz, flags) = p
        dt = 0.0
        if self.last_t_us is not None:
            dt = (t_us - self.last_t_us) * 1e-6
        self.last_t_us = t_us

        # crude vertical estimator: baro altitude + complementary velocity
        if flags & protocol.FLAG_BARO:
            h_baro = 44330.0 * (1.0 - (hpa / self.base_hpa) ** 0.1903)
            if dt > 0:
                v_baro = (h_baro - self.h) / max(dt, 1e-6)
                self.v = 0.7 * self.v + 0.3 * v_baro
            self.h = h_baro
        # net vertical accel for launch detect (sensor convention: az = -g at rest)
        net_a = az + GRAVITY
        if dt > 0 and not flags & protocol.FLAG_BARO:
            self.v += net_a * dt
            self.h += self.v * dt

        self._update_phase(t_us, net_a)

        if self.phase == PHASES["COASTING"]:
            self.target_opening, self.est_apogee, self.apogee_err = guidance_update(
                self.h, self.v, hpa, temp_c, self.actuator.opening,
                TARGET_APOGEE_M, GUIDANCE_KP)
        else:
            self.target_opening = 0.0

        if dt > 0:
            self.actuator.step(self.target_opening, dt)

        return struct.pack(
            protocol.STATUS_FMT, t_us, self.phase,
            self.h, self.v, self.tilt,
            self.est_apogee, self.apogee_err,
            self.target_opening * ACT_MAX_POS_CM,
            self.actuator.opening * ACT_MAX_POS_CM,
            0.0, 0.0,   # kfP00, kfP11 (not simulated by virtual Teensy)
            0.0,        # actDutyPct
        )

    def _update_phase(self, t_us, net_a):
        if self.phase == PHASES["IDLE"]:
            if net_a >= LAUNCH_ACCEL:
                self.confirm += 1
                if self.confirm >= LAUNCH_CONFIRM:
                    self.phase = PHASES["LAUNCHED"]
                    self.launch_time_us = t_us
            else:
                self.confirm = 0
        elif self.phase == PHASES["LAUNCHED"]:
            if t_us - self.launch_time_us >= COAST_DELAY_US:
                self.phase = PHASES["COASTING"]
        elif self.phase == PHASES["COASTING"]:
            if self.v <= 0.0:
                self.phase = PHASES["APOGEE"]
        elif self.phase == PHASES["APOGEE"]:
            self.phase = PHASES["DESCENT"]


def main():
    master, slave = os.openpty()
    tty.setraw(master)
    print(f"Virtual Teensy listening. In another terminal run:")
    print(f"  python scripts/run_hil.py --port {os.ttyname(slave)}")
    sys.stdout.flush()

    vt = VirtualTeensy()
    parser = protocol.FrameParser()

    while True:
        data = os.read(master, 4096)
        if not data:
            break
        for pkt_type, payload in parser.feed(data):
            if pkt_type == protocol.PKT_INIT:
                vt = VirtualTeensy()
                vt.base_hpa, _temp = struct.unpack(protocol.INIT_FMT, payload)
                os.write(master, protocol.make_frame(protocol.PKT_INIT_ACK, b"\x01"))
                print(f"INIT received: base pressure {vt.base_hpa:.2f} hPa")
            elif pkt_type == protocol.PKT_SENSOR:
                fields = struct.unpack(protocol.SENSOR_FMT, payload)
                status = vt.on_sensor(fields)
                os.write(master, protocol.make_frame(protocol.PKT_STATUS, status))


if __name__ == "__main__":
    main()
