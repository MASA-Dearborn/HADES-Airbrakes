# HADES Airbrakes — Hardware-in-the-Loop Simulator

RocketPy 6DOF ascent simulation closed-loop coupled to the HADES flight
computer (Teensy 4.1) over USB serial. The Python side simulates the vehicle,
atmosphere and sensors; the Teensy runs the **unmodified flight stack**
(Madgwick + Kalman estimation, state machine, apogee-prediction guidance,
actuator PID) against injected sensor data and reports its actuator position
back, which the simulation applies as airbrake deployment.



**Lock-step, paced to real time.** Each RocketPy controller call (200 Hz of
sim time = the IMU rate) sends one sensor packet and blocks for the Teensy's
reply. In `HIL_MODE` the firmware scheduler, coast timers and estimator run
on the packet timestamps, not `micros()`. By default the host additionally
holds each packet until the wall clock reaches its sim timestamp, so **1 s of
sim time = 1 s of real time** — required when the physical actuator is in the
loop, since it moves in wall time. `--no-real-time` restores
fastest-possible lock-step (results for the simulated-actuator build are
identical either way; only the wall duration changes).

## Setup

From the HADES-Airbrakes repository root, flash the safe simulated-actuator HIL
firmware:

```bash
# Default HIL: sensors AND actuator simulated (no hardware required).
# Deterministic; use this for guidance/estimator tuning and apogee numbers.
pio run -e teensy41-hil -t upload

# Real-actuator HIL: sensors injected, but the real motor + Hall encoder are
# in the loop (actual speed, friction, dead-band, homing). Wire the H-bridge
# (IN1=36, IN2=37) and encoder (Hall A=34, B=35).
pio run -e teensy41-hil-act -t upload
```

Then install the simulator dependencies. On Windows PowerShell:

```powershell
cd simulator
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

On macOS or Linux:

```bash
cd simulator
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

The host (`run_hil.py`) talks the same protocol to both — it reads the
reported `actualPosCm` and doesn't know whether the actuator is real or
modelled.

## Usage

Close PlatformIO Serial Monitor first. Only one program can own the Teensy's
serial port at a time.

```bash
# Pure-software run, no Teensy needed (uncontrolled + python guidance port):
python scripts/run_offline.py

# Full HIL flight (autodetects a connected Teensy):
python scripts/run_hil.py --with-reference

# Windows may also specify the known port directly:
python scripts/run_hil.py --port COM7 --no-real-time --with-reference

# Protocol/pipeline test without hardware (two terminals):
python scripts/virtual_teensy.py          # prints a pty path
python scripts/run_hil.py --port /dev/ttysNNN

# Unit tests:
pytest tests/
```

Results (CSV logs, plots, summary.json) land in `results/<timestamp>_<tag>/`.

## What a HIL run does

1. **Handshake** — Python sends `INIT` with pad pressure/temperature from the
   simulated atmosphere; the Teensy uses it as its baro calibration base, then
   homes the (simulated) actuator.
2. **Pad settling** — `--settle` seconds (default 8) of static sensor packets
   at the rail attitude so Madgwick/Kalman converge, like the real pad wait.
3. **Flight** — RocketPy integrates; every 5 ms of sim time one sensor packet
   goes out and the returned actuator position drives the airbrake drag
   (CFD Cd(deployment, Mach) table, identical to `guidance.cpp`).
4. **Report** — apogee error vs the 3048 m target, firmware estimates vs
   truth, deployment history; optionally compared against an offline
   python-guidance reference and an uncontrolled flight.

## Actuator in the loop

The airbrake linear actuator can be either modelled in firmware or physically
driven, selected at build time:

| Build (`pio run -e ...`) | Sensors | Actuator | Use for |
|---|---|---|---|
| `teensy41-hil`     | injected | first-order plant (`hilPlantAdvance`, advanced on sim time) | guidance/estimator tuning, apogee accuracy |
| `teensy41-hil-act` | injected | **real motor + Hall encoder** | actuator PID, dead-band, friction, homing |
| `test-actuator`    | none     | real motor + encoder (bench, no flight) | calibrating the constants below |

**Time base for `teensy41-hil-act`.** A physical actuator moves in wall-clock
time, so the host paces the lock-step loop to real time by default: each
packet (5 ms sim) is held until the wall clock reaches its sim timestamp,
keeping the actuator's real motion on the same clock the sim samples it with.
Pacing is anchored absolutely (jitter doesn't accumulate); if the serial
round-trip + integration can't sustain 5 ms/packet the run falls behind and
the host warns (`lag_warn_s`, default 50 ms) — the per-packet lag is logged as
`rt_lag_ms` in the CSV and the max is printed at the end. If a run shows
sustained lag, its actuator timing is distorted by that amount, so treat the
apogee number accordingly. `--no-real-time` disables pacing (fine for the
simulated-actuator build, which advances its plant on sim time and is
deterministic regardless of host speed).

**Bench calibration.** With the motor + encoder wired:

```bash
cd ..
pio test -e test-actuator --filter calibrate_actuator
```

It homes, checks PID tracking to ¼/½/full stroke, and prints `[CAL]` lines for
the no-load speed and full-stroke transition count. Update `config.h`
(`HIL_ACT_MAX_SPEED_CMS` if speed differs >20%; `ACTUATOR_TRANSITIONS_FULL_STROKE`
if count differs >5%) and mirror the speed into `src/hades_hil/config.py`
(`ACT_MAX_SPEED_CMS`) so the default-HIL and offline plants match the hardware.

## Conventions (must match firmware)

- **Accelerometer sign**: the firmware expects `az = -g` at rest upright
  (see `estimation.cpp: verticalAccel`). The sensor model therefore sends
  `a_raw = R_world→body · (a_world − [0, 0, +9.80665])`.
- **Altitude**: firmware `h` is AGL (baro-relative to the INIT base
  pressure); RocketPy `z` is ASL.
- Constants mirrored from `config.h` are annotated in
  `src/hades_hil/config.py` and `guidance_ref.py` — keep them in sync by hand.
- Wire protocol spec: header of `../include/hil.h` (mirrored in
  `src/hades_hil/protocol.py`).

## Known gaps / notes

- Magnetometer is sent as zeros with its valid flag off (firmware only logs it).
- `override_rocket_drag=True`: the CFD table (its 0-deployment column) replaces
  the rocket drag CSVs whenever the brakes surface is evaluated, including
  during the burn. The table is full-vehicle Cd, so this keeps the plant
  consistent with the onboard predictor.
- World-frame acceleration is finite-differenced from the integrator's
  velocity output between controller calls (5 ms) — smooth dense output makes
  this accurate, and sensor noise is added on top.
- The HIL actuator plant (`HIL_ACT_MAX_SPEED_CMS` in `config.h`) should be set
  to the bench-measured actuator speed — see **Actuator in the loop** above for
  the calibration test that measures it.
- Atmosphere defaults to ISA (offline-friendly). Set
  `EnvConfig.atmosphere="forecast"|"ensemble"` + `date` for GFS/GEFS like the
  Vulcan notebook (needs `netCDF4` + network).
