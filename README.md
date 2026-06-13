# HADES Airbrakes — Flight Firmware (Teensy 4.1)

Closed-loop airbrakes flight computer. Reads IMU + barometer, estimates
attitude and vertical state, predicts apogee, and drives a linear-actuator
airbrake to hit a target apogee (3048 m AGL). Runs a fixed-rate cooperative
scheduler (`main.cpp`) on simulated or wall-clock time depending on build.

## Key Modules

- `attitude`   — Madgwick orientation estimation (gyro + accel), tilt angle
- `kalman`     — 2-state vertical Kalman filter, x = [h, v]ᵀ
- `estimation` — sensor-fusion pipeline (IMU predict + barometer correct)
- `sensors`    — hardware drivers for IMU (BMI088), baro (BMP585), mag (LIS2MDL)
- `statemachine` — IDLE → LAUNCHED → COASTING → APOGEE → DESCENT (+ FAULT)
- `guidance`   — energy-balance apogee predictor + CFD Cd table + P controller
- `actuator`   — H-bridge + Hall-encoder linear actuator, homing, position PID
- `logging`    — binary flight log to SD (`FLTxx.BIN`)
- `hil`        — hardware-in-the-loop serial protocol (see HIL sim repo)
- `types` / `config` — shared structs, constants, and tuning parameters

--------------------------------------------------

## Estimation Overview

**Attitude:** quaternion, gyro integration + accel correction (accel gated to
0.5–1.5 g so boost/free-fall don't corrupt it). No magnetometer fusion — yaw
drifts, but only tilt is used for the deploy inhibit.

**Vertical:** 2-state Kalman filter. Predict from IMU-derived world-vertical
acceleration at 200 Hz; correct from barometric altitude at 50 Hz.

**Guidance:** predicts apogee from (h, v) via an energy balance with a
mean-drag correction, looks up Cd from a Mach × deployment CFD table, and
commands brake opening proportional to the predicted overshoot.

--------------------------------------------------

## Build & Upload

Install the PlatformIO CLI, then pick an environment (`platformio.ini`):

```bash
pio run -e teensy41              -t upload   # flight build (real sensors + actuator)
pio run -e teensy41-hil          -t upload   # HIL: sensors + actuator simulated by host
pio run -e teensy41-hil-act      -t upload   # HIL: virtual sensors, REAL motor + encoder
pio test -e test-actuator --filter calibrate_actuator   # bench calibration (real motor)
```

The HIL builds pair with the Python simulator in `~/hades_airbrakes_control`
(`HADES Airbrakes — Hardware-in-the-Loop Simulator`); in `HIL_MODE` all flight
logic runs unmodified on the Teensy while sensors/actuator are driven over USB.

--------------------------------------------------

## Flight Readiness

**Status: simulation-validated, NOT yet flight-cleared.** All four builds
compile; the estimation/guidance/control logic is verified in HIL (apogee
within ~1 m of target). The following must be closed before flight:

**Blocking — safety**
- [ ] **Sensor-failure fail-safe + watchdog.** `readIMU()` does not yet flag a
      failed/stuck read, and there is no watchdog, so a sensor hang freezes the
      loop with brakes at their last position. Need: hardware watchdog (fails
      safe to retracted on a hang) + IMU validity check that latches `FAULT`
      and retracts the brakes after N consecutive bad reads. (Maybe?)
- [ ] **Confirm brake fail-safe and recovery independence.** Define brake
      behaviour on fault/reset/brownout (retract), and confirm parachute
      recovery is on a fully independent system (an airbrakes failure must not
      threaten recovery).
- [ ] **Coast-delay margin.** `SM_COAST_DELAY_US` (4.4 s) equals the N3300R
      burn time — add margin so brakes cannot deploy under residual thrust.

**Blocking — validation**
- [ ] **One real-hardware run** (ground/captive-carry or tumble-and-tap) to
      confirm the estimator converges and launch detect doesn't false-trigger
      on real BMI088 noise. HIL cannot exercise this.

**Accuracy (won't fail the flight, will miss target)**
- [ ] Confirm the CFD Cd table matches as-built brake geometry.
- [ ] Consider a dynamic-pressure / Mach limit on deployment (guidance commands
      full extension on predicted overshoot, including just after burnout).

**Pre-flight bench checks**
- [ ] Verify the real BMI088 reads `az ≈ −g` upright (the whole estimator
      assumes this sign convention).
- [ ] Run `calibrate_actuator` and update `ACTUATOR_TRANSITIONS_FULL_STROKE`,
      `ACTUATOR_FULL_STROKE_CM`, `HIL_ACT_MAX_SPEED_CMS`.

> Note: the current motor controller has no nSLEEP/Reset/FAULT lines, so those
> pins are disabled in `config.h`. `FAULT` is therefore reachable only via a
> software fault (see the first blocking item).

--------------------------------------------------

## Steps

0. Theory, module testing [✓]
1. Sensor Interface [✓]  — TODO: SPI support (I²C path is the default)
2. State Estimation [✓]
3. Actuator Driver/Sensor/Controller [✓]
4. Guidance Controller [✓] — CFD drag table integrated
5. State Machine [✓]
6. Safety & Inhibit Logic [~] — tilt-lock + coast inhibit done; watchdog /
   sensor-failure fault path outstanding (see Flight Readiness)
7. RF telemetry [ ] (Canceled)
8. Logging & HIL Testing [✓]

--------------------------------------------------
## Documentation:

- HIL simulator + protocol: `~/hades_airbrakes_control/README.md`
- Wire protocol spec: `include/hil.h`

--------------------------------------------------
## Contributors

Contributors are listed alphabetically. Roles indicate primary areas of contribution.

### MADA-D - Project HADES 2026 – Airbrakes Electrical Team

Marcus Wada – Project Lead  
Andrew Bellinger – Theory & Testing  
Ali Beidoun – CFD & Assembly  
Osman Hadi – PCB Design & Schematics  
Resul Ilmammedov – Hardware Prototyping  
David Richardson – Theory  

### Essential Support & Leadership

Robert Everitt – Electrical Chief Engineer  
Michael Miney – Mechanical Chief Engineer (CFD)  
Chase Sutherlin – Airbrakes Mechanical Design  
Karishma Patnaik - Assistant Professor
