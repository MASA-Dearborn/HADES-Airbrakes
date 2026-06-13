#pragma once
#include <cstdint>
#include <Arduino.h>
#include "types.h"

// Hardware-in-the-loop support.
//
// Build with -DHIL_MODE (env:teensy41-hil) to replace physical sensors and
// the actuator plant with values exchanged over USB serial with the host
// simulator (~/hades_airbrakes_control).  Flight logic — estimator, state
// machine, guidance, actuator PID — runs unmodified.
//
// Wire protocol (little-endian, framed):
//   [0xAA][0x55][type][len][payload ...][crc8]
// crc8: poly 0x07, init 0x00, computed over type+len+payload.
// ASCII debug prints may appear between frames; 0xAA never occurs in ASCII
// so the host parser skips them safely.
//
// Host -> Teensy
//   0x01 INIT    : float basePressure_hPa, float baseTempC
//   0x02 SENSOR  : uint32 simTimeUs,
//                  float ax, ay, az,        (m/s², body frame, sensor sign convention)
//                  float gx, gy, gz,        (rad/s, body frame)
//                  float hpa, tempC,
//                  float mx, my, mz,        (µT)
//                  uint8 flags              (bit0 IMU, bit1 baro, bit2 mag valid)
// Teensy -> Host
//   0x81 INIT_ACK: uint8 protocolVersion
//   0x82 STATUS  : uint32 simTimeUs (echo of last sensor packet),
//                  uint8 phase,
//                  float estH, estV, tiltDeg,
//                  float estApogeeM, apogeeErrM,
//                  float targetPosCm, actualPosCm,
//                  float kfP00, kfP11,        (Kalman altitude/velocity variance)
//                  float actDutyPct           (actuator PID duty cycle, -100..100)

// timeNowUs() is the clock for all flight logic (scheduler, state machine,
// log timestamps).  Real builds: micros().  HIL builds: simulated time
// carried by the last SENSOR packet, so coast timers, estimator dt and the
// task scheduler all advance in simulation time.
#ifdef HIL_MODE
uint32_t hilTimeUs();
inline uint32_t timeNowUs() { return hilTimeUs(); }
#else
inline uint32_t timeNowUs() { return micros(); }
#endif

#ifdef HIL_MODE

#define HIL_PROTOCOL_VERSION 2

#define HIL_SYNC1 0xAA
#define HIL_SYNC2 0x55

#define HIL_PKT_INIT     0x01
#define HIL_PKT_SENSOR   0x02
#define HIL_PKT_INIT_ACK 0x81
#define HIL_PKT_STATUS   0x82

#define HIL_FLAG_IMU  0x01
#define HIL_FLAG_BARO 0x02
#define HIL_FLAG_MAG  0x04

// Blocks until the host INIT packet arrives, then replies INIT_ACK.
void hilInit();

// Pump the serial parser.  Returns true once per newly completed SENSOR
// packet — the main loop runs one scheduler pass per packet.
bool hilPoll();

// Base atmosphere from the INIT packet (pad pressure / temperature).
float hilBasePressureHpa();
float hilBaseTempC();

// Virtual sensor reads, called by the HIL implementations in sensors.cpp.
void hilReadIMU(SensorData& d);
void hilReadBaro(SensorData& d);
void hilReadMag(SensorData& d);

void hilSendStatus(const StateEstimate& state,
                   const GuidanceState& guidance,
                   FlightPhase          phase,
                   float                actualPosCm,
                   float                kfP00,
                   float                kfP11,
                   float                actDutyPct);

// HIL actuator plant model — only when the real actuator is NOT wired up.
#ifndef HIL_REAL_ACTUATOR
void actuatorHilStep(uint32_t simNowUs);   // advance plant in sim time
void actuatorHilStepDt(float dtS);         // advance by explicit dt (homing loop)
#endif

#endif // HIL_MODE
