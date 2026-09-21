#!/usr/bin/env python3
"""Run the 6DOF simulation without hardware: uncontrolled flight plus a
flight controlled by the Python port of the firmware guidance law.

    python scripts/run_offline.py
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hades_hil.config import SimConfig
from hades_hil.runner import (make_results_dir, run_offline, save_report,
                              summarize)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uncontrolled-only", action="store_true")
    args = ap.parse_args()

    cfg = SimConfig()
    runs = []

    if not args.uncontrolled_only:
        print("[offline] controlled flight (python guidance port)...")
        flight, bridge, env = run_offline(cfg, controlled=True)
        runs.append((summarize(flight, env, bridge.log, "controlled"), bridge.log))

    print("[offline] uncontrolled flight (brakes locked closed)...")
    f_unc, b_unc, e_unc = run_offline(cfg, controlled=False)
    runs.append((summarize(f_unc, e_unc, b_unc.log, "uncontrolled"), b_unc.log))

    out = make_results_dir("offline")
    save_report(out, runs)


if __name__ == "__main__":
    main()
