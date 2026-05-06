#pragma once

#include <Arduino.h>

//constants
// #define PI 3.14159f
#define GRAVITY 9.80665f

//calibration
#define BARO_CALIB_SAMPLES 50
//Serial
//Timing
//I2C Addresses
//Actuator Pins
//Actuator Settings
//Safety Limits

//State Estimation Gains
#define KF_SIGMA_A2 2.73e-3f
#define KF_R 5.71e-3f

#define MADGWICK_BETA 0.1f

//Control Gains
//GUidance Limits