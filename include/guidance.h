#pragma once
#include "types.h"

class Guidance {
public:
    void begin();

    // currentOpeningFraction: actual flap position from actuator [0, 1].
    // Returns GuidanceState with estimated apogee, error, and target position.
    GuidanceState update(const StateEstimate& state,
                         const SensorData&    sensors,
                         float                currentOpeningFraction);

private:
    float lookupCd(float mach, float openingFraction) const;
    float predictApogee(float h, float v, float Cd, float rho) const;
};
