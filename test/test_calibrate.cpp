// Actuator calibration tests — run with:
//   pio test -e test-actuator --filter calibrate_actuator
//
// Requires a real Teensy 4.1 with the motor, H-bridge, and Hall encoder wired
// (no sensors needed).  Tests verify homing, PID position tracking, and
// measure the two config constants you may need to tune:
//
//   HIL_ACT_MAX_SPEED_CMS         — update if [CAL] speed differs by >20%
//   ACTUATOR_TRANSITIONS_FULL_STROKE — update if [CAL] count differs by >5%

#include <Arduino.h>
#include <unity.h>
#include "actuator.h"
#include "config.h"

// ── helpers ──────────────────────────────────────────────────────────────────

// Run the PID loop at the configured period until within tolerance of target
// or timeout elapses.  Returns true on success.
static bool driveToTarget(float targetCm, uint32_t timeoutMs) {
    float clamped = constrain(targetCm, ACTUATOR_MIN_POSITION_CM, ACTUATOR_MAX_POSITION_CM);
    actuatorSetTarget(targetCm);
    actuatorSetEnabled(true);
    uint32_t startMs  = millis();
    uint32_t lastUs   = micros();
    while ((uint32_t)(millis() - startMs) < timeoutMs) {
        if ((uint32_t)(micros() - lastUs) >= ACTUATOR_CONTROL_PERIOD_US) {
            lastUs += ACTUATOR_CONTROL_PERIOD_US;
            actuatorUpdatePID();
        }
        if (fabsf(actuatorGetPositionCm() - clamped) <= ACTUATOR_POSITION_TOLERANCE_CM)
            return true;
    }
    actuatorSetEnabled(false);
    return false;
}

// Drive to the physical hard stop by commanding a target well beyond the
// config limit so the PID saturates at max duty.  Waits until the encoder
// stops changing (stall for 400 ms) or a 15 s safety timeout.
// Returns settled position (cm).  Sets *elapsedMs to motion duration.
static float driveToPhysicalStop(float farTargetCm, uint32_t *elapsedMs) {
    actuatorSetTarget(farTargetCm);
    actuatorSetEnabled(true);
    float    lastPos    = actuatorGetPositionCm();
    uint32_t lastMoveMs = millis();
    uint32_t startMs    = millis();
    uint32_t lastUs     = micros();

    while (true) {
        if ((uint32_t)(micros() - lastUs) >= ACTUATOR_CONTROL_PERIOD_US) {
            lastUs += ACTUATOR_CONTROL_PERIOD_US;
            actuatorUpdatePID();
        }
        float pos = actuatorGetPositionCm();
        if (fabsf(pos - lastPos) > ACTUATOR_CM_PER_TRANSITION * 0.5f) {
            lastPos    = pos;
            lastMoveMs = millis();
        }
        if ((uint32_t)(millis() - lastMoveMs) >= 400) break;   // stalled at hard stop
        if ((uint32_t)(millis() - startMs)    > 15000) break;  // safety
    }
    actuatorSetEnabled(false);
    if (elapsedMs) *elapsedMs = millis() - startMs;
    return actuatorGetPositionCm();
}

// ── setUp / tearDown ─────────────────────────────────────────────────────────

void setUp() {
    // Home once on first test; tearDown retracts to 0 between tests.
    static bool inited = false;
    if (!inited) {
        actuatorInit();
        actuatorHome();
        inited = true;
    }
}

void tearDown() {
    driveToTarget(0.0f, 8000);
    actuatorSetEnabled(false);
}

// ── tests ────────────────────────────────────────────────────────────────────

void test_homing_completes() {
    actuatorInit();
    actuatorHome();
    TEST_ASSERT_TRUE_MESSAGE(actuatorIsHomed(), "actuatorHome() did not set homed flag");
    TEST_ASSERT_FLOAT_WITHIN(ACTUATOR_POSITION_TOLERANCE_CM, 0.0f, actuatorGetPositionCm());
}

void test_pid_reaches_quarter_stroke() {
    float target = ACTUATOR_MAX_POSITION_CM * 0.25f;
    bool  ok     = driveToTarget(target, 6000);
    TEST_ASSERT_TRUE_MESSAGE(ok, "PID did not reach 2.5 cm within 6 s");
    TEST_ASSERT_FLOAT_WITHIN(ACTUATOR_POSITION_TOLERANCE_CM, target, actuatorGetPositionCm());
}

void test_pid_reaches_half_stroke() {
    float target = ACTUATOR_MAX_POSITION_CM * 0.5f;
    bool  ok     = driveToTarget(target, 6000);
    TEST_ASSERT_TRUE_MESSAGE(ok, "PID did not reach 5 cm within 6 s");
    TEST_ASSERT_FLOAT_WITHIN(ACTUATOR_POSITION_TOLERANCE_CM, target, actuatorGetPositionCm());
}

void test_pid_reaches_full_stroke() {
    bool ok = driveToTarget(ACTUATOR_MAX_POSITION_CM, 8000);
    TEST_ASSERT_TRUE_MESSAGE(ok, "PID did not reach full stroke within 8 s");
    TEST_ASSERT_FLOAT_WITHIN(ACTUATOR_POSITION_TOLERANCE_CM * 2.0f,
                             ACTUATOR_MAX_POSITION_CM, actuatorGetPositionCm());
}

void test_pid_retracts_to_zero() {
    // Extend first so retraction is exercised.
    driveToTarget(ACTUATOR_MAX_POSITION_CM, 8000);
    bool ok = driveToTarget(0.0f, 8000);
    TEST_ASSERT_TRUE_MESSAGE(ok, "PID did not retract to 0 within 8 s");
    TEST_ASSERT_FLOAT_WITHIN(ACTUATOR_POSITION_TOLERANCE_CM, 0.0f, actuatorGetPositionCm());
}

// Measures no-load extension speed at PID saturation.
// Prints the result so you can compare against HIL_ACT_MAX_SPEED_CMS.
// Update HIL_ACT_MAX_SPEED_CMS in config.h if the printed value differs by >20%.
void test_measure_extension_speed() {
    driveToTarget(0.0f, 5000);   // ensure retracted

    uint32_t elapsedMs = 0;
    // PID will saturate immediately (target 50 cm >> actual), driving at max duty.
    // The PID clamps the target to ACTUATOR_MAX_POSITION_CM internally, so the
    // motor runs at full speed until the position reaches the config limit.
    float finalPos  = driveToPhysicalStop(50.0f, &elapsedMs);
    float speedCms  = (elapsedMs > 200) ? (finalPos / (elapsedMs * 1e-3f)) : 0.0f;

    Serial.printf("[CAL] extension speed: %.2f cm/s  "
                  "(config HIL_ACT_MAX_SPEED_CMS = %.1f cm/s)\n",
                  speedCms, (float)HIL_ACT_MAX_SPEED_CMS);
    Serial.printf("[CAL] reached %.3f cm in %lu ms\n", finalPos, elapsedMs);

    TEST_ASSERT_GREATER_THAN_MESSAGE(0.5f, speedCms,
        "Actuator did not move — check motor wiring and power");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(HIL_ACT_MAX_SPEED_CMS * 0.5f,
                                     HIL_ACT_MAX_SPEED_CMS, speedCms,
        "Measured speed differs by >50% from HIL_ACT_MAX_SPEED_CMS — update config.h");
}

// Measures Hall encoder transitions across the configured full stroke.
// Prints the result so you can compare against ACTUATOR_TRANSITIONS_FULL_STROKE.
// Update that constant in config.h if the printed count differs by >5%.
//
// Note: the PID clamps the target to ACTUATOR_MAX_POSITION_CM, so this
// measures transitions to the config limit, not the physical hard stop.
// If you need to calibrate TRANSITIONS_FULL_STROKE from scratch, temporarily
// set ACTUATOR_MAX_POSITION_CM to a large value (e.g. 20.0), rebuild, and run.
void test_measure_full_stroke_transitions() {
    driveToTarget(0.0f, 5000);   // ensure retracted

    uint32_t elapsedMs = 0;
    float    stopPos   = driveToPhysicalStop(50.0f, &elapsedMs);

    // positionCount cancels through ACTUATOR_CM_PER_TRANSITION — gives raw encoder count.
    long measured = lroundf(stopPos / ACTUATOR_CM_PER_TRANSITION);

    Serial.printf("[CAL] full-stroke transitions: %ld  "
                  "(config ACTUATOR_TRANSITIONS_FULL_STROKE = %ld)\n",
                  measured, (long)ACTUATOR_TRANSITIONS_FULL_STROKE);
    Serial.printf("[CAL] settled position: %.3f cm  in %lu ms\n", stopPos, elapsedMs);

    TEST_ASSERT_GREATER_THAN_MESSAGE(0L, measured,
        "No transitions counted — check Hall encoder wiring");
    TEST_ASSERT_INT_WITHIN_MESSAGE(
        (int)(ACTUATOR_TRANSITIONS_FULL_STROKE * 0.10f),
        (int)ACTUATOR_TRANSITIONS_FULL_STROKE,
        (int)measured,
        "Transition count differs by >10% from config — update "
        "ACTUATOR_TRANSITIONS_FULL_STROKE in config.h");
}

// ── test runner ──────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(500);

    UNITY_BEGIN();
    RUN_TEST(test_homing_completes);
    RUN_TEST(test_pid_reaches_quarter_stroke);
    RUN_TEST(test_pid_reaches_half_stroke);
    RUN_TEST(test_pid_reaches_full_stroke);
    RUN_TEST(test_pid_retracts_to_zero);
    RUN_TEST(test_measure_extension_speed);
    RUN_TEST(test_measure_full_stroke_transitions);
    UNITY_END();
}

void loop() { delay(1000); }
