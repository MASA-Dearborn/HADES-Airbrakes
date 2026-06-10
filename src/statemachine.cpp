#include "statemachine.h"
#include "config.h"
#include <Arduino.h>

void StateMachine::begin() {
    phase        = FlightPhase::IDLE;
    launchTimeUs = 0;
    confirmCount = 0;
    lastTiltDeg  = 0.0f;
}

void StateMachine::update(const StateEstimate& state) {
    lastTiltDeg = state.attitude.tiltDeg;

    switch (phase) {

        case FlightPhase::IDLE:
            // Require SM_LAUNCH_CONFIRM_COUNT consecutive samples above threshold
            // to reject vibration spikes on the pad.
            if (state.vertical.a >= SM_LAUNCH_ACCEL_THRESHOLD_MS2) {
                if (++confirmCount >= SM_LAUNCH_CONFIRM_COUNT) {
                    phase        = FlightPhase::LAUNCHED;
                    launchTimeUs = micros();
                    confirmCount = 0;
                }
            } else {
                confirmCount = 0;
            }
            break;

        case FlightPhase::LAUNCHED:
            // Motor burnout is not directly observable, so coast phase begins
            // at a fixed time after launch to guarantee brakes only deploy post-burn
            if ((uint32_t)(micros() - launchTimeUs) >= SM_COAST_DELAY_US) {
                phase = FlightPhase::COASTING;
            }
            break;

        case FlightPhase::COASTING:
            // Apogee detected when vertical velocity crosses zero
            // Kalman smoothing keeps this from flickering
            if (state.vertical.v <= 0.0f) {
                phase = FlightPhase::APOGEE;
            }
            break;

        case FlightPhase::APOGEE:
            // One-tick transitional state so logging can record the apogee event
            phase = FlightPhase::DESCENT;
            break;

        case FlightPhase::DESCENT:
        case FlightPhase::FAULT:
            // Terminal states — estimator and logger keep running; no transitions out
            break;
    }
}

FlightPhase StateMachine::getPhase() const {
    return phase;
}

bool StateMachine::canDeployBrakes() const {
    return phase == FlightPhase::COASTING &&
           lastTiltDeg <= SM_TILT_LOCK_DEG;
}

void StateMachine::triggerFault() {
    phase = FlightPhase::FAULT;
}

const char* phaseName(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::IDLE:     return "IDLE";
        case FlightPhase::LAUNCHED: return "LAUNCHED";
        case FlightPhase::COASTING: return "COASTING";
        case FlightPhase::APOGEE:   return "APOGEE";
        case FlightPhase::DESCENT:  return "DESCENT";
        case FlightPhase::FAULT:    return "FAULT";
        default:                    return "UNKNOWN";
    }
}
