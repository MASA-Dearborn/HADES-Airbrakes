#pragma once

//constants
// #define PI 3.14159f
#define GRAVITY 9.80665f

//calibration
#define BARO_CALIB_SAMPLES 50
#define SEALEVEL_HPA 1013.25f
//Serial
//Timing
//I2C Addresses
#define USE_SPI_SENSORS 0
//Actuator Pins
//Actuator Settings
//Safety Limits

//State Estimation Gains
#define KF_SIGMA_A2 2.73e-3f
#define KF_R 5.71e-3f

#define MADGWICK_BETA 0.1f

//Control Gains
//GUidance Limits