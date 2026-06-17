#include "guidance.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

// --- CFD drag table ---------------------------------------------------
// Rows  : Mach number breakpoints (8)
// Cols  : flap opening fraction breakpoints — 0, 1/3, 2/3, 1 (4)
//
// Cd values from CFD at 0°, 25°, 50°, 75° flap deflection.

static const float MACH_AXIS[8] = {
    0.25f, 0.50f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.00f
};
static const float OPEN_AXIS[4] = {
    0.0f, 0.3333f, 0.6667f, 1.0f
};
static const float CD_TABLE[8][4] = {
    { 0.336f, 0.594f, 1.012f, 1.298f },  // Mach 0.25
    { 0.304f, 0.581f, 1.051f, 1.352f },  // Mach 0.50
    { 0.289f, 0.624f, 1.202f, 1.529f },  // Mach 0.75
    { 0.287f, 0.647f, 1.254f, 1.583f },  // Mach 0.80
    { 0.286f, 0.686f, 1.318f, 1.649f },  // Mach 0.85
    { 0.288f, 0.754f, 1.415f, 1.740f },  // Mach 0.90
    { 0.338f, 0.854f, 1.596f, 1.908f },  // Mach 0.95
    { 0.450f, 1.013f, 1.805f, 2.177f },  // Mach 1.00
};

static constexpr int N_MACH = 8;
static constexpr int N_OPEN = 4;

// Bilinear interpolation.  Inputs clamped to table bounds.
float Guidance::lookupCd(float mach, float opening) const {
    mach    = constrain(mach,    MACH_AXIS[0], MACH_AXIS[N_MACH - 1]);
    opening = constrain(opening, OPEN_AXIS[0], OPEN_AXIS[N_OPEN - 1]);

    // Lower bounding index along each axis
    int mi = N_MACH - 2;
    for (int i = 0; i < N_MACH - 1; i++) {
        if (mach <= MACH_AXIS[i + 1]) { mi = i; break; }
    }

    int oi = N_OPEN - 2;
    for (int i = 0; i < N_OPEN - 1; i++) {
        if (opening <= OPEN_AXIS[i + 1]) { oi = i; break; }
    }

    // Normalised fractional distance within the cell
    float tm = (mach    - MACH_AXIS[mi]) / (MACH_AXIS[mi + 1] - MACH_AXIS[mi]);
    float to = (opening - OPEN_AXIS[oi]) / (OPEN_AXIS[oi + 1] - OPEN_AXIS[oi]);

    float c00 = CD_TABLE[mi    ][oi    ];
    float c10 = CD_TABLE[mi + 1][oi    ];
    float c01 = CD_TABLE[mi    ][oi + 1];
    float c11 = CD_TABLE[mi + 1][oi + 1];

    return c00 * (1.0f - tm) * (1.0f - to)
         + c10 * tm          * (1.0f - to)
         + c01 * (1.0f - tm) * to
         + c11 * tm          * to;
}

// Energy-based apogee predictor with mean-drag correction.
//
// Derivation — energy balance from current state (h, v) to apogee (h_apex, 0):
//   0.5 m v² = m g (h_apex − h) + W_drag
//
// Drag work approximated by using average velocity = v/2 (linear decrease v→0):
//   W_drag ≈ 0.5 · ρ · (v/2)² · Cd · A · (h_apex − h)
//           = ρ Cd A v² (h_apex − h) / 8
//
// Solving for h_apex:
//   h_apex = h + v² / (2g + ρ Cd A v² / (4m))
float Guidance::predictApogee(float h, float v, float Cd, float rho) const {
    if (v <= 0.0f) return h;

    float k  = (rho * Cd * ROCKET_REF_AREA_M2) / (4.0f * ROCKET_MASS_KG);
    float dh = (v * v) / (2.0f * GRAVITY + k * v * v);
    return h + dh;
}

// --- Public API -------------------------------------------------------

void Guidance::begin() {}

GuidanceState Guidance::update(const StateEstimate& state,
                                const SensorData&    sensors,
                                float                currentOpening) {
    GuidanceState out   = {};
    out.timeUs          = state.vertical.timeUs;

    float v = state.vertical.v;
    float h = state.vertical.h;

    // Atmospheric properties from barometer
    float T_K        = sensors.tempC + 273.15f;
    float P_Pa       = sensors.hpa   * 100.0f;
    float rho        = P_Pa / (287.05f * T_K);
    float soundSpeed = 20.05f * sqrtf(T_K);
    float mach       = (v > 0.0f && soundSpeed > 0.0f) ? (v / soundSpeed) : 0.0f;

    // Predict apogee at current drag setting
    float Cd                 = lookupCd(mach, currentOpening);
    out.estimatedApogeeM     = predictApogee(h, v, Cd, rho);
    out.apogeeErrorM         = out.estimatedApogeeM - ROCKET_TARGET_APOGEE_M;

    // Proportional controller: positive error (overshoot) → open brakes
    float targetOpening      = GUIDANCE_KP * out.apogeeErrorM;
    targetOpening            = constrain(targetOpening, 0.0f, 1.0f);
    out.targetPositionCm     = targetOpening * ACTUATOR_MAX_POSITION_CM;

    
    
    return out;
}
