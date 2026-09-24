"""Simulation configuration.

Vehicle/motor numbers come from the Vulcan notebook
(~/vulcan_airbrakes_control/notebooks/simulation.ipynb); the constants that
mirror the firmware are annotated with their config.h counterparts and must
be kept in sync by hand.
"""

from dataclasses import dataclass, field
from pathlib import Path

# --- Constants shared with the firmware (config.h) -------------------------

GRAVITY = 9.80665                  # GRAVITY

IMU_RATE_HZ = 200                  # IMU_PERIOD_US = 5000
IMU_DT_US = 1_000_000 // IMU_RATE_HZ
BARO_EVERY_N = 4                   # BARO_PERIOD_US = 20000 -> 50 Hz

ACT_MAX_POS_CM = 5.0              # ACTUATOR_MAX_POSITION_CM
ACT_MAX_SPEED_CMS = 0.9            # HIL_ACT_MAX_SPEED_CMS

ROCKET_MASS_KG = 33.135            # ROCKET_MASS_KG (firmware predictor mass)
ROCKET_REF_AREA_M2 = 0.01928       # ROCKET_REF_AREA_M2
TARGET_APOGEE_M = 3048.0           # ROCKET_TARGET_APOGEE_M
GUIDANCE_KP = 0.02                 # GUIDANCE_KP

DATA_DIR = Path(__file__).resolve().parents[2] / "data"
RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"


@dataclass
class EnvConfig:
    latitude: float = 43.26327778
    longitude: float = -86.03233333
    elevation: float = 207.953
    # "standard" needs no network.  "forecast"/"ensemble" pull GFS/GEFS from
    # NOAA NOMADS OpenDAP; "windy" pulls GFS/ECMWF/ICON from the Windy API.
    # Use "windy" when NOMADS OpenDAP is down (it returns a 1e7 Pa sentinel).
    # All non-"standard" options require internet.
    atmosphere: str = "standard"
    date: tuple | None = (2026, 9, 26, 12)   # (Y, M, D, hour UTC) for forecast models
    forecast_file: str = "GFS"               # windy: GFS/ECMWF/ICON/ICONEU


@dataclass
class MotorConfig:
    eng_file: Path = DATA_DIR / "AeroTech_N3300R.eng"
    dry_mass: float = 7.338
    dry_inertia: tuple = (0.4, 0.4, 0.02)
    nozzle_radius: float = 2.737 * 0.0254 / 2
    grain_number: int = 6
    grain_density: float = 1730.0
    grain_outer_radius: float = 3.365 * 0.0254 / 2
    grain_initial_inner_radius: float = 1.25 * 0.0254 / 2
    grain_initial_height: float = 6 * 0.0254
    grain_separation: float = 1 / 16 * 0.0254
    grains_center_of_mass_position: float = 0.566
    center_of_dry_mass_position: float = 0.523
    nozzle_position: float = 0.0
    burn_time: float = 4.4
    throat_radius: float = 1 * 0.0254 / 2


@dataclass
class RocketConfig:
    radius: float = 15.7 / 200
    mass: float = 26.2                # without motor
    inertia: tuple = (4.449, 4.449, 1)
    # power-off (coast) drag comes from the CFD table, see cd_table.closed_brakes_drag_curve
    power_on_drag: Path = DATA_DIR / "power_on_drag.CSV"
    nose_length: float = 92.7 / 100
    nose_kind: str = "von karman"
    nose_position: float = 2.01
    fin_n: int = 4
    fin_root_chord: float = 30.5 / 100
    fin_tip_chord: float = 10.2 / 100
    fin_span: float = 7 * 2.54 / 100
    fin_sweep_length: float = 17.8 / 100
    fin_position: float = -125 / 100
    tail_top_radius: float = 6.17 * 2.54 / 200
    tail_bottom_radius: float = 2.0 * 2.54 / 100
    tail_length: float = 14 / 100
    tail_position: float = -157 / 100
    motor_position: float = -163.6 / 100


@dataclass
class RailConfig:
    length: float = 4.166
    inclination: float = 84.0
    heading: float = 133.0


@dataclass
class NoiseConfig:
    seed: int | None = 7
    accel_std: float = 0.05            # m/s² per axis (BMI088)
    accel_bias_std: float = 0.02       # constant per-axis bias drawn at init
    gyro_std: float = 0.002            # rad/s per axis
    gyro_bias_std: float = 0.001
    baro_std_hpa: float = 0.03         # BMP585-class
    temp_std_c: float = 0.05


@dataclass
class HilConfig:
    port: str | None = None            # None -> autodetect Teensy USB serial
    baud: int = 115200                 # ignored by Teensy USB CDC, required by pyserial
    settle_s: float = 8.0              # static pad time before liftoff so the
                                       # Madgwick/Kalman filters converge
    timeout_s: float = 5.0
    handshake_timeout_s: float = 30.0
    real_time: bool = True             # pace SENSOR packets to the wall clock so
                                       # 1 s of sim time = 1 s of real time; a
                                       # physical actuator (teensy41-hil-act)
                                       # moves in wall time, so this keeps its
                                       # motion consistent with the sim clock
    lag_warn_s: float = 0.05           # warn when the sim falls this far behind
                                       # the wall clock (serial/integration too slow)


@dataclass
class SimConfig:
    env: EnvConfig = field(default_factory=EnvConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    rocket: RocketConfig = field(default_factory=RocketConfig)
    rail: RailConfig = field(default_factory=RailConfig)
    noise: NoiseConfig = field(default_factory=NoiseConfig)
    hil: HilConfig = field(default_factory=HilConfig)
    sampling_rate: int = IMU_RATE_HZ   # one controller call = one IMU packet
    terminate_on_apogee: bool = True
    max_time: float = 120.0
