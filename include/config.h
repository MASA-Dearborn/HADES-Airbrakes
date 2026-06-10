#pragma once

//constants
#define GRAVITY 9.80665f



//calibration
#define BARO_CALIB_SAMPLES 50
#define SEALEVEL_HPA 1013.25f
//Serial
//Timing
//I2C Addresses
#define USE_SPI_SENSORS 1

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
#define ACTUATOR_NSLEEP_PIN 35
#define ACTUATOR_IN1_PIN    36
#define ACTUATOR_IN2_PIN    37

#define HALL_A_PIN 23
#define HALL_B_PIN 22

#define ACTUATOR_FAULT_PIN 14

//PWM Settings

#define ACTUATOR_PWM_FREQ_HZ 1000

#define ACTUATOR_PWM_MAX 255
#define ACTUATOR_PWM_MIN_EXTEND 150
#define ACTUATOR_PWM_MIN_RETRACT 150

#define ACTUATOR_EXTEND_SIGN 1
#define ACTUATOR_RETRACT_SIGN -1

//Actuator Settings
#define ACTUATOR_FULL_STROKE_CM 10.0f //4.5? about
#define ACTUATOR_TRANSITIONS_FULL_STROKE 4938.0f //should I make it changable because of environment 
#define ACTUATOR_CM_PER_TRANSITION \
    (ACTUATOR_FULL_STROKE_CM / ACTUATOR_TRANSITIONS_FULL_STROKE)

#define ACTUATOR_MAX_POSITION_CM 10.0f
#define ACTUATOR_MIN_POSITION_CM 0.0f

#define ACTUATOR_POSITION_TOLERANCE_CM 0.1f

//Homing
#define ACTUATOR_HOMING_PWM -255

#define ACTUATOR_HOMING_TIMEOUT_MS 8000UL
#define ACTUATOR_HOMING_STALL_TIME_MS 300UL

#define ACTUATOR_HOMING_INTERVAL_MS 50000000UL


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


//GUidance Limits
