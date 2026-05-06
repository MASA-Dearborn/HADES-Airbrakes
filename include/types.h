#pragma once

#include <Arduino.h>

struct SensorData{
  float ax, ay, az; //m/s^2
  float gx, gy, gz; //rad/s
  uint32_t imuTimeUs;
  bool imuUpdated; 

  float mx, my, mz; //uT
  uint32_t magTimeUs;
  bool magUpdated; 

  float hpa; //hPa
  float tempC; //C
  uint32_t baroTimeUs;
  bool baroUpdated; 
};

struct StateEstimate{
  float h;
  float v;
  float a;

  float e1;
  float e2;
  float e3;
  float e4;

  uint32_t estimateTimeUs;
};

struct ControlCmd{
  float t_cm; //target in cm
  float u; //actuator input
  bool enableActuator;
  uint32_t controlTimeUs;

};