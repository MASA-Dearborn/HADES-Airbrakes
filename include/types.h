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
    float h, v, a;
    uint32_t timeUs;
};

struct AttitudeState {
    
    float q0, q1, q2, q3;
    float tiltDeg;

    uint32_t timeUs;
};

struct StateEstimate {
    
    AttitudeState attitude;
    VerticalState vertical;

};

// struct ControlCmd{
//     float targetCm; //target in cm
//     float pwmCmd; //actuator input
//     bool enableActuator;
//     uint32_t controlTimeUs;

// };

// struct ActuatorState {
//     float positionCm;
//     long positionCount;
//     long transitionCount;
//     int motorDirection;
//     bool homed;
//     bool homingActive;
//     uint32_t timeUs;
// };

struct GuidanceState {
    float estimatedApogeeM;
    float apogeeErrorM;
    float targetPositionCm;
    uint32_t timeUs;
};



