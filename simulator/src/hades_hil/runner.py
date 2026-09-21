"""Flight orchestration: HIL run, offline reference run, results/plots."""

import datetime
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rocketpy import Flight

from .config import RESULTS_DIR, TARGET_APOGEE_M, SimConfig
from .controller import HilBridge, OfflineBridge
from .hil_link import HilLink, autodetect_port
from .sensors import SensorSimulator
from .vehicle import add_airbrakes, build_environment, build_motor, build_rocket


def _fly(cfg: SimConfig, env, controller_function):
    motor = build_motor(cfg)
    rocket = build_rocket(cfg, motor)
    add_airbrakes(rocket, controller_function, cfg.sampling_rate)
    return Flight(
        rocket=rocket,
        environment=env,
        rail_length=cfg.rail.length,
        inclination=cfg.rail.inclination,
        heading=cfg.rail.heading,
        terminate_on_apogee=cfg.terminate_on_apogee,
        max_time=cfg.max_time,
    )


def run_offline(cfg: SimConfig, controlled: bool = True):
    """Reference flight with the Python port of the guidance law
    (controlled=True) or brakes locked closed (controlled=False)."""
    env = build_environment(cfg)
    bridge = OfflineBridge(env, controlled=controlled)
    flight = _fly(cfg, env, bridge.controller)
    return flight, bridge, env


def run_hil(cfg: SimConfig):
    """Full hardware-in-the-loop flight against the Teensy."""
    env = build_environment(cfg)
    sensor_sim = SensorSimulator(env, cfg.noise)

    port = cfg.hil.port or autodetect_port()
    print(f"[hil] connecting to {port}")
    link = HilLink(port, cfg.hil.baud, timeout_s=cfg.hil.timeout_s)
    try:
        print("[hil] handshaking (sends pad atmosphere, waits for Teensy boot)...")
        link.handshake(sensor_sim.pad_pressure_hpa(), sensor_sim.pad_temp_c(),
                       timeout_s=cfg.hil.handshake_timeout_s)
        print("[hil] handshake ok, starting flight")
        if cfg.hil.real_time:
            print("[hil] real-time pacing on: 1 s sim time = 1 s wall clock")

        bridge = HilBridge(link, sensor_sim, settle_s=cfg.hil.settle_s,
                           real_time=cfg.hil.real_time,
                           lag_warn_s=cfg.hil.lag_warn_s)
        flight = _fly(cfg, env, bridge.controller)
        if cfg.hil.real_time:
            print(f"[hil] real-time pacing: max lag "
                  f"{bridge.rt_max_lag_s * 1e3:.1f} ms behind wall clock")
    finally:
        link.close()
    return flight, bridge, env


# --- Results ----------------------------------------------------------------

def make_results_dir(tag: str) -> Path:
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = RESULTS_DIR / f"{stamp}_{tag}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def summarize(flight, env, log, label: str) -> dict:
    apogee_agl = float(flight.apogee) - float(env.elevation)
    deployments = [d for d in log.column("deployment") if not math.isnan(d)]
    return {
        "label": label,
        "apogee_agl_m": apogee_agl,
        "target_apogee_m": TARGET_APOGEE_M,
        "apogee_error_m": apogee_agl - TARGET_APOGEE_M,
        "apogee_time_s": float(flight.apogee_time),
        "max_deployment": max(deployments) if deployments else 0.0,
    }


def save_report(out_dir: Path, runs: list):
    """runs: list of (summary_dict, FlightLog).  First entry is the main run."""
    summaries = [s for s, _ in runs]
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summaries, f, indent=2)

    for summary, log in runs:
        log.save_csv(out_dir / f"log_{summary['label']}.csv")

    _plot_altitude(out_dir, runs)
    _plot_control(out_dir, runs[0])
    print(f"\nResults in {out_dir}")
    for s in summaries:
        print(f"  {s['label']:>12}: apogee {s['apogee_agl_m']:7.1f} m AGL "
              f"(target {s['target_apogee_m']:.0f}, error {s['apogee_error_m']:+.1f} m, "
              f"max deploy {s['max_deployment'] * 100:.0f}%)")


def _plot_altitude(out_dir: Path, runs):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for summary, log in runs:
        t = log.column("t")
        ax1.plot(t, log.column("z_agl"), label=f"{summary['label']} (truth)")
        ax2.plot(t, log.column("vz"), label=f"{summary['label']} (truth)")
    # firmware estimates exist only for the main (HIL) run
    main_log = runs[0][1]
    if any(not math.isnan(v) for v in main_log.column("est_h")):
        ax1.plot(main_log.column("t"), main_log.column("est_h"),
                 "--", label="firmware est")
        ax2.plot(main_log.column("t"), main_log.column("est_v"),
                 "--", label="firmware est")
    ax1.axhline(TARGET_APOGEE_M, color="k", ls=":", label="target apogee")
    ax1.set_ylabel("altitude AGL [m]")
    ax2.set_ylabel("vertical velocity [m/s]")
    ax2.set_xlabel("flight time [s]")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "altitude_velocity.png", dpi=150)
    plt.close(fig)


def _plot_control(out_dir: Path, run):
    summary, log = run
    t = log.column("t")
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11), sharex=True)

    ax1.plot(t, log.column("deployment"), label="deployment level")
    target = [p / 10.0 for p in log.column("target_pos_cm")]
    ax1.plot(t, target, "--", alpha=0.7, label="commanded (target pos / stroke)")
    ax1.set_ylabel("deployment [0–1]")
    ax1.set_ylim(-0.05, 1.05)

    ax2.plot(t, log.column("est_apogee_m"), label="predicted apogee")
    ax2.axhline(TARGET_APOGEE_M, color="k", ls=":", label="target")
    ax2.axhline(summary["apogee_agl_m"], color="g", ls="--", label="achieved")
    ax2.set_ylabel("apogee [m AGL]")

    # Kalman covariance (left axis) + actuator duty (right axis)
    import math
    p00 = log.column("kf_p00")
    p11 = log.column("kf_p11")
    duty = log.column("act_duty_pct")
    valid = [not math.isnan(v) for v in p00]
    if any(valid):
        tv = [t[i] for i in range(len(t)) if valid[i]]
        ax3.plot(tv, [p00[i] for i in range(len(t)) if valid[i]],
                 label="P00 (alt var)")
        ax3.plot(tv, [p11[i] for i in range(len(t)) if valid[i]],
                 label="P11 (vel var)")
    ax3.set_ylabel("Kalman covariance")
    ax3_r = ax3.twinx()
    valid_d = [not math.isnan(v) for v in duty]
    if any(valid_d):
        tv_d = [t[i] for i in range(len(t)) if valid_d[i]]
        ax3_r.plot(tv_d, [duty[i] for i in range(len(t)) if valid_d[i]],
                   color="tab:red", alpha=0.6, label="duty %")
    ax3_r.set_ylabel("actuator duty [%]")
    ax3_r.set_ylim(-110, 110)
    ax3.set_xlabel("flight time [s]")

    lines1, labels1 = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_r.get_legend_handles_labels()
    ax3.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    for ax in (ax1, ax2, ax3):
        ax.grid(alpha=0.3)
        if ax is not ax3:
            ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "control.png", dpi=150)
    plt.close(fig)
