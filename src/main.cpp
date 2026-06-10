#include "Arduino.h"
#include "config.h"
#include "sensors.h"
#include "estimation.h"
#include "actuator.h"
#include "statemachine.h"
#include "guidance.h"
#include "logging.h"

// --- Module-level state -----------------------------------------------

static float         basePressure_hPa;
static SensorData    data          = {};
static StateEstimate state         = {};
static GuidanceState guidanceState = {};
static Estimator     estimator;
static StateMachine  sm;
static Guidance      guidance;

static FlightPhase   phase     = FlightPhase::IDLE;
static FlightPhase   prevPhase = FlightPhase::IDLE;

// --- Task timestamps --------------------------------------------------
// Each task fires when (uint32_t)(micros() - lastXUs) >= its period.
// Always increment by the fixed period (never reassign to now)
// so drift does not accumulate over time.

static uint32_t lastImuUs   = 0;
static uint32_t lastBaroUs  = 0;
static uint32_t lastMagUs   = 0;
static uint32_t lastInnerUs = 0;
static uint32_t lastOuterUs = 0;
static uint32_t lastLogUs   = 0;
static uint32_t lastPrintUs = 0;

// --- Forward declarations ---------------------------------------------

static void printDebug();

// --- Setup ------------------------------------------------------------

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    sensorsInit();
    basePressure_hPa = calibrateBaroBase();
    Serial.print("Base pressure: ");
    Serial.println(basePressure_hPa);

    estimator.begin(basePressure_hPa);
    sm.begin();
    guidance.begin();
    loggerBegin();

    // actuatorInit();
    // actuatorHome();

    // Stagger first-fire times so no two tasks coincide in the first loop pass
    uint32_t now = micros();
    lastImuUs    = now;
    lastBaroUs   = now + 3000;
    lastMagUs    = now + 1000;
    lastInnerUs  = now +  500;
    lastOuterUs  = now + 7000;
    lastLogUs    = now + 11000;
    lastPrintUs  = now + 15000;
}

// --- Loop -------------------------------------------------------------

void loop() {
    uint32_t now = micros();

    // 200 Hz — IMU read, Madgwick attitude update, Kalman predict
    if ((uint32_t)(now - lastImuUs) >= IMU_PERIOD_US) {
        lastImuUs += IMU_PERIOD_US;

        data.imuUpdated = false;
        readIMU(data);
        estimator.update(data);
        state = estimator.getState();
    }

    // 50 Hz — barometer read, Kalman correction
    if ((uint32_t)(now - lastBaroUs) >= BARO_PERIOD_US) {
        lastBaroUs += BARO_PERIOD_US;

        data.baroUpdated = false;
        readBaro(data);
        estimator.update(data);
        state = estimator.getState();
    }

    // 100 Hz — magnetometer read (logged only; no fusion until mag calibration added)
    if ((uint32_t)(now - lastMagUs) >= MAG_PERIOD_US) {
        lastMagUs += MAG_PERIOD_US;

        data.magUpdated = false;
        readMag(data);
    }

    // 100 Hz — inner control: actuator position PID
    if ((uint32_t)(now - lastInnerUs) >= ACTUATOR_CONTROL_PERIOD_US) {
        lastInnerUs += ACTUATOR_CONTROL_PERIOD_US;

        actuatorUpdatePID();

        // if (!digitalRead(ACTUATOR_FAULT_PIN)) sm.triggerFault();
    }

    // 20 Hz — outer control: state machine + guidance + actuator enable/disable
    if ((uint32_t)(now - lastOuterUs) >= OUTER_CTRL_PERIOD_US) {
        lastOuterUs += OUTER_CTRL_PERIOD_US;

        sm.update(state);
        phase = sm.getPhase();

        // Flush SD on transition to a terminal state to protect data before landing
        if (phase != prevPhase) {
            if (phase == FlightPhase::DESCENT || phase == FlightPhase::FAULT) {
                loggerFlush();
            }
            prevPhase = phase;
        }

        if (sm.canDeployBrakes()) {
            float opening = actuatorGetPositionCm() / ACTUATOR_MAX_POSITION_CM;
            guidanceState = guidance.update(state, data, opening);
            actuatorSetTarget(guidanceState.targetPositionCm);
            actuatorSetEnabled(true);
        } else if (phase == FlightPhase::COASTING || phase == FlightPhase::APOGEE) {
            // Tilt lock or apogee: retract to zero
            actuatorSetTarget(0.0f);
            actuatorSetEnabled(true);
        } else {
            // IDLE / LAUNCHED / DESCENT / FAULT: motor off
            actuatorSetEnabled(false);
        }
    }

    // 50 Hz — SD logging
    if ((uint32_t)(now - lastLogUs) >= LOG_PERIOD_US) {
        lastLogUs += LOG_PERIOD_US;

        loggerWrite(state, data, guidanceState, phase, actuatorGetPositionCm());
    }

    // 10 Hz — serial debug output
    if ((uint32_t)(now - lastPrintUs) >= DEBUG_PRINT_PERIOD_US) {
        lastPrintUs += DEBUG_PRINT_PERIOD_US;
        printDebug();
    }
}

// --- Debug output (10 Hz, serial only) --------------------------------

static void printDebug() {
    Serial.print("PHASE: "); Serial.println(phaseName(phase));

    Serial.print("IMU | Acc: ");
    Serial.print(data.ax, 2); Serial.print(", ");
    Serial.print(data.ay, 2); Serial.print(", ");
    Serial.print(data.az, 2);
    Serial.print(" | Gyro: ");
    Serial.print(data.gx, 2); Serial.print(", ");
    Serial.print(data.gy, 2); Serial.print(", ");
    Serial.print(data.gz, 2);
    Serial.println();

    Serial.print("ATT | q: ");
    Serial.print(state.attitude.q0, 3); Serial.print(", ");
    Serial.print(state.attitude.q1, 3); Serial.print(", ");
    Serial.print(state.attitude.q2, 3); Serial.print(", ");
    Serial.print(state.attitude.q3, 3);
    Serial.print(" | tilt: "); Serial.print(state.attitude.tiltDeg, 1); Serial.println(" deg");

    Serial.print("BARO | ");
    Serial.print(data.hpa, 2); Serial.print(" hPa  ");
    Serial.print(data.tempC, 1); Serial.println(" C");

    Serial.print("VERT | h: "); Serial.print(state.vertical.h, 1);
    Serial.print(" m  v: ");    Serial.print(state.vertical.v, 1);
    Serial.print(" m/s  a: ");  Serial.print(state.vertical.a, 1); Serial.println(" m/s^2");

    Serial.print("GUID | est: "); Serial.print(guidanceState.estimatedApogeeM, 0);
    Serial.print(" m  err: ");    Serial.print(guidanceState.apogeeErrorM, 0);
    Serial.print(" m  tgt: ");    Serial.print(guidanceState.targetPositionCm, 1);
    Serial.print(" cm  act: ");   Serial.print(actuatorGetPositionCm(), 1); Serial.println(" cm");

    Serial.print("LOG | ready: "); Serial.println(loggerReady());
    Serial.println("========================");
}
