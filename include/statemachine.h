#pragma once
#include "types.h"

class StateMachine {
public:
    void begin();
    void update(const StateEstimate& state);
    FlightPhase getPhase() const;

    // True only when brakes are allowed to deploy:
    // must be COASTING and tilt within limits
    bool canDeployBrakes() const;

    // Call from anywhere a hard fault is detected
    // (motor fault pin, sensor failure, etc.)
    void triggerFault();

private:
    FlightPhase phase        = FlightPhase::IDLE;
    uint32_t    launchTimeUs = 0;
    int         confirmCount = 0;
    float       lastTiltDeg  = 0.0f;
};

const char* phaseName(FlightPhase phase);
