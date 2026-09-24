"""RocketPy vehicle/environment builders (Vulcan + AeroTech N3300R)."""

from rocketpy import Environment, Rocket, SolidMotor

from .cd_table import rocketpy_drag_curve
from .config import SimConfig


def build_environment(cfg: SimConfig) -> Environment:
    env = Environment(
        latitude=cfg.env.latitude,
        longitude=cfg.env.longitude,
        elevation=cfg.env.elevation,
    )
    if cfg.env.atmosphere in ("forecast", "ensemble", "windy"):
        if cfg.env.date is None:
            raise ValueError(
                "env.date is required for forecast/ensemble/windy atmospheres")
        env.set_date(cfg.env.date)
        model_type = {
            "forecast": "Forecast",   # NOAA NOMADS OpenDAP (GFS/NAM/RAP)
            "ensemble": "Ensemble",   # NOAA NOMADS OpenDAP (GEFS)
            "windy": "Windy",         # Windy API (GFS/ECMWF/ICON) — use when
                                      # NOMADS OpenDAP is down
        }[cfg.env.atmosphere]
        env.set_atmospheric_model(type=model_type, file=cfg.env.forecast_file)
    # default: ISA standard atmosphere, no network required
    return env


def build_motor(cfg: SimConfig) -> SolidMotor:
    m = cfg.motor
    return SolidMotor(
        thrust_source=str(m.eng_file),
        dry_mass=m.dry_mass,
        dry_inertia=m.dry_inertia,
        nozzle_radius=m.nozzle_radius,
        grain_number=m.grain_number,
        grain_density=m.grain_density,
        grain_outer_radius=m.grain_outer_radius,
        grain_initial_inner_radius=m.grain_initial_inner_radius,
        grain_initial_height=m.grain_initial_height,
        grain_separation=m.grain_separation,
        grains_center_of_mass_position=m.grains_center_of_mass_position,
        center_of_dry_mass_position=m.center_of_dry_mass_position,
        nozzle_position=m.nozzle_position,
        burn_time=m.burn_time,
        throat_radius=m.throat_radius,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )


def build_rocket(cfg: SimConfig, motor: SolidMotor) -> Rocket:
    r = cfg.rocket
    rocket = Rocket(
        radius=r.radius,
        mass=r.mass,
        inertia=r.inertia,
        center_of_mass_without_motor=0,
        power_off_drag=str(r.power_off_drag),
        power_on_drag=str(r.power_on_drag),
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_nose(length=r.nose_length, kind=r.nose_kind, position=r.nose_position)
    rocket.add_trapezoidal_fins(
        n=r.fin_n,
        root_chord=r.fin_root_chord,
        tip_chord=r.fin_tip_chord,
        span=r.fin_span,
        sweep_length=r.fin_sweep_length,
        position=r.fin_position,
        cant_angle=0,
    )
    rocket.add_tail(
        top_radius=r.tail_top_radius,
        bottom_radius=r.tail_bottom_radius,
        length=r.tail_length,
        position=r.tail_position,
    )
    rocket.add_motor(motor, position=r.motor_position)
    return rocket


def add_airbrakes(rocket: Rocket, controller_function, sampling_rate: int):
    """Attach the airbrakes surface driven by `controller_function`.

    The CFD table is full-vehicle Cd, so override_rocket_drag=True: whenever
    the simulation evaluates the brakes (including deployment 0) the table
    replaces the rocket's own drag curve.  The 0-opening column is the
    clean-configuration Cd from the same CFD campaign, keeping the plant
    consistent with the firmware predictor.
    """
    return rocket.add_air_brakes(
        drag_coefficient_curve=rocketpy_drag_curve,
        controller_function=controller_function,
        sampling_rate=sampling_rate,
        clamp=True,
        override_rocket_drag=True,
        initial_observed_variables=[0.0, 0.0, 0.0],
        name="HADES Airbrakes",
    )
