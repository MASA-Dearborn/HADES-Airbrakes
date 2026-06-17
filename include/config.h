#pragma once

//constants
#define GRAVITY 9.80665f



//calibration
#define BARO_CALIB_SAMPLES 50
#define SEALEVEL_HPA 1013.25f
//Serial
//Timing
//I2C Addresses
#define USE_SPI_SENSORS 0

#define NRF_CE_PIN       4
#define NRF_CS_PIN       6

// Sensor chip selects
#define BMP_CS_PIN       7
#define LIS_CS_PIN       8
#define BMI_ACC_CS_PIN   9
#define BMI_GYR_CS_PIN   10

// SPI pins, Teensy default SPI
#define SPI_MOSI_PIN     11
#define SPI_MISO_PIN     12
#define SPI_SCK_PIN      13

//Actuator Pins
#define ACTUATOR_IN1_PIN    36
#define ACTUATOR_IN2_PIN    37

#define HALL_A_PIN 34 //green
#define HALL_B_PIN 35 ///yellow
//blue vcc
// orange gnd
//black motor-
//red motor+



//PWM Settings

#define ACTUATOR_PWM_FREQ_HZ 1000

#define ACTUATOR_PWM_MAX 255
#define ACTUATOR_PWM_MIN_EXTEND 150
#define ACTUATOR_PWM_MIN_RETRACT 150

#define ACTUATOR_EXTEND_SIGN 1
#define ACTUATOR_RETRACT_SIGN -1

//Actuator Settings
#define ACTUATOR_FULL_STROKE_CM 5.0f //4.5? about
#define ACTUATOR_TRANSITIONS_FULL_STROKE 4938.0f //should I make it changable because of environment 
#define ACTUATOR_CM_PER_TRANSITION \
    (ACTUATOR_FULL_STROKE_CM / ACTUATOR_TRANSITIONS_FULL_STROKE)

#define ACTUATOR_MAX_POSITION_CM 10.0f
#define ACTUATOR_MIN_POSITION_CM 0.0f

#define ACTUATOR_POSITION_TOLERANCE_CM 0.1f

//Homing
#define ACTUATOR_HOMING_PWM 255

#define ACTUATOR_HOMING_TIMEOUT_MS 15000UL
#define ACTUATOR_HOMING_STALL_TIME_MS 300UL

#define ACTUATOR_HOMING_POLL_MS 10UL


//Safety Limits

//State Estimation Gains
#define KF_SIGMA_A2 2.73e-3f
#define KF_R 5.71e-3f

#define MADGWICK_BETA 0.1f

//Control Gains
#define ACTUATOR_KP 70.0f
#define ACTUATOR_KI 1.0f

#define ACTUATOR_CONTROL_PERIOD_US 10000UL
#define ACTUATOR_CONTROL_DT_S 0.01f

#define ACTUATOR_DUTY_MAX 100.0f
#define ACTUATOR_DUTY_MIN -100.0f


// Task scheduling periods
#define IMU_PERIOD_US           5000UL     // 200 Hz — Madgwick + Kalman predict
#define BARO_PERIOD_US          20000UL    //  50 Hz — Kalman baro update
#define MAG_PERIOD_US           10000UL    // 100 Hz — logged only, no fusion yet
#define OUTER_CTRL_PERIOD_US    50000UL    //  20 Hz — state machine + guidance
#define LOG_PERIOD_US           20000UL    //  50 Hz — SD write
#define DEBUG_PRINT_PERIOD_US   100000UL   //  10 Hz — serial output

// SD logging
#define LOG_PREALLOC_BYTES      (10UL * 1024UL * 1024UL)  // 10 MB pre-alloc to avoid mid-flight cluster delays

// State Machine
#define SM_LAUNCH_ACCEL_THRESHOLD_MS2  30.0f     // net vertical accel to confirm launch (~3 g above gravity)
#define SM_LAUNCH_CONFIRM_COUNT        5          // consecutive IMU samples required to latch launch
#define SM_COAST_DELAY_US              4400000UL  // time after launch before entering COASTING (4.52 s)
#define SM_TILT_LOCK_DEG               30.0f      // max tilt angle for airbrake deployment

// Guidance — rocket physical properties (must be set before flight)
#define ROCKET_MASS_KG         33.135f      // kg  — post-burnout dry mass
#define ROCKET_REF_AREA_M2     0.01928f  // m²  — body cross-section (15.67 cm diameter)
#define ROCKET_TARGET_APOGEE_M 3048.0f  // m   — AGL target apogee

// Guidance P gain: opening_fraction = Kp * (predicted_apogee - target_apogee)
// At Kp = 0.02: 50 m overshoot -> full extension.  Tune on bench first.
#define GUIDANCE_KP            0.02f     // 1/m

// HIL actuator plant model (only used when built with -DHIL_MODE).
// Match HIL_ACT_MAX_SPEED_CMS to the bench-measured no-load speed at
// full PWM so closed-loop deployment timing is realistic.
#define HIL_ACT_MAX_SPEED_CMS  4.0f      // cm/s at PWM 255
#define HIL_ACT_START_POS_CM   2.0f      // physical position at boot, exercises homing
