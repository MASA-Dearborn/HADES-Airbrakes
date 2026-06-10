#pragma once
#include <cstdint>

enum class FlightPhase : uint8_t {
    IDLE = 0,
    LAUNCHED,
    COASTING,
    APOGEE,
    DESCENT,
    FAULT
};

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

struct GuidanceState {
    float estimatedApogeeM;
    float apogeeErrorM;
    float targetPositionCm;
    uint32_t timeUs;
};



