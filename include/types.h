#pragma once
#include <cstdint>

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

struct VerticalState {
    float h;
    float v;
    float a;
    uint32_t timeUs;
};

struct AttitudeState {
    
    float q0;
    float q1;
    float q2;
    float q3;
  
    uint32_t timeUs;
};

struct StateEstimate {
    
    AttitudeState attitude;
    VerticalState vertical;

};

struct ControlCmd{
    float t_cm; //target in cm
    float u; //actuator input
    bool enableActuator;
    uint32_t controlTimeUs;

};