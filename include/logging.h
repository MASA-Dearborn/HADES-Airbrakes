#pragma once
#include "types.h"

// Binary log record written to SD at LOG_PERIOD_US.
// Packed to eliminate alignment padding — fields are populated sequentially
// on write, never read back in flight, so unaligned access cost is irrelevant.
struct __attribute__((packed)) LogRecord {
    uint32_t timeUs;

    uint8_t  phase;

    // Raw IMU (m/s², rad/s)
    float ax, ay, az;
    float gx, gy, gz;

    // Raw magnetometer (µT)
    float mx, my, mz;

    // Raw barometer
    float hpa, tempC;

    // Vertical state estimate
    float h, v, a;

    // Attitude estimate
    float q0, q1, q2, q3;
    float tiltDeg;

    // Guidance
    float estimatedApogeeM;
    float apogeeErrorM;
    float targetPositionCm;

    // Actuator
    float actualPositionCm;
};

// loggerBegin() opens the next available FLTxx.BIN on the SD card.
// Returns false and prints to Serial if SD init or file open fails.
// Flight continues without logging — never blocks on SD failure.
bool loggerBegin();

void loggerWrite(const StateEstimate& state,
                 const SensorData&    sensors,
                 const GuidanceState& guidance,
                 FlightPhase          phase,
                 float                actualPositionCm);

// Force a flush to physical media.  Call on phase transitions to
// DESCENT or FAULT to protect data before potential power loss.
void loggerFlush();

bool loggerReady();
