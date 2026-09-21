"""Per-exchange flight log (sim truth + Teensy telemetry) and CSV export."""

import csv
import math
from pathlib import Path

COLUMNS = [
    "t", "sim_time_us",
    "z_agl", "vz", "speed", "mach",
    "deployment",
    "phase",
    "est_h", "est_v", "tilt_deg",
    "est_apogee_m", "apogee_err_m",
    "target_pos_cm", "actual_pos_cm",
    "kf_p00", "kf_p11",
    "act_duty_pct",
    "rt_lag_ms",
]


class FlightLog:
    def __init__(self):
        self.rows = []

    def add(self, **kwargs):
        self.rows.append({c: kwargs.get(c, math.nan) for c in COLUMNS})

    def column(self, name):
        return [row[name] for row in self.rows]

    def save_csv(self, path: Path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(self.rows)
        return path
