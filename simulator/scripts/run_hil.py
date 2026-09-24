#!/usr/bin/env python3
"""Run a hardware-in-the-loop flight against the Teensy.

Flash the HIL firmware first:
    pio run -e teensy41-hil -t upload

Then:
    cd simulator
    python scripts/run_hil.py [--port COM7] [--with-reference]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hades_hil.config import SimConfig
from hades_hil.runner import (make_results_dir, run_hil, run_offline,
                              save_report, summarize)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None,
                    help="serial port (default: autodetect Teensy USB serial; "
                         "Windows example: COM7)")
    ap.add_argument("--settle", type=float, default=8.0,
                    help="pad settling time in sim seconds (default 8)")
    ap.add_argument("--seed", type=int, default=7, help="sensor noise seed")
    ap.add_argument("--no-real-time", action="store_true",
                    help="run lock-step as fast as possible instead of pacing "
                         "packets to the wall clock; fine for the simulated-"
                         "actuator build, but a physical actuator "
                         "(teensy41-hil-act) needs real-time pacing")
    ap.add_argument("--with-reference", action="store_true",
                    help="also run offline python-guidance and uncontrolled "
                         "reference flights for comparison")
    args = ap.parse_args()

    cfg = SimConfig()
    cfg.hil.port = args.port
    cfg.hil.settle_s = args.settle
    cfg.hil.real_time = not args.no_real_time
    cfg.noise.seed = args.seed

    flight, bridge, env = run_hil(cfg)
    runs = [(summarize(flight, env, bridge.log, "hil"), bridge.log)]

    if args.with_reference:
        print("[ref] running offline python-guidance reference...")
        f_ref, b_ref, e_ref = run_offline(cfg, controlled=True)
        runs.append((summarize(f_ref, e_ref, b_ref.log, "reference"), b_ref.log))
        print("[ref] running uncontrolled reference...")
        f_unc, b_unc, e_unc = run_offline(cfg, controlled=False)
        runs.append((summarize(f_unc, e_unc, b_unc.log, "uncontrolled"), b_unc.log))

    out = make_results_dir("hil")
    save_report(out, runs)


if __name__ == "__main__":
    main()
