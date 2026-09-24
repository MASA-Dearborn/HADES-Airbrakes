#include "actuator.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include "types.h"
#include "hil.h"


static ActuatorState s_act = {};
static ControlCmd    s_cmd = {};

#if defined(HIL_MODE) && !defined(HIL_REAL_ACTUATOR)
// --- HIL actuator plant -------------------------------------------------
// First-order plant: position rate proportional to PWM command, hard stops
// at 0 and full stroke.  Drives s_act.positionCount through the same encoder
// counts the hardware would produce, so PID, homing stall detection and
// actuatorGetPositionCm() are exercised unchanged.

static float s_hilPosCm     = HIL_ACT_START_POS_CM;  // physical position
static float s_hilZeroCm    = 0.0f;                  // physical pos at last encoder zero
static uint32_t s_hilLastUs = 0;
static bool     s_hilClockValid = false;

static void hilPlantAdvance(float dtS) {
    float v = (s_cmd.pwmCmd / (float)ACTUATOR_PWM_MAX) * HIL_ACT_MAX_SPEED_CMS;
    s_hilPosCm += v * dtS;
    if (s_hilPosCm < 0.0f)                     s_hilPosCm = 0.0f;
    if (s_hilPosCm > ACTUATOR_FULL_STROKE_CM)  s_hilPosCm = ACTUATOR_FULL_STROKE_CM;

    long count = lroundf((s_hilPosCm - s_hilZeroCm) / ACTUATOR_CM_PER_TRANSITION);
    noInterrupts();
    if (count != s_act.positionCount) s_act.transitionCount += labs(count - s_act.positionCount);
    s_act.positionCount = count;
    interrupts();
}

void actuatorHilStepDt(float dtS) {
    hilPlantAdvance(dtS);
}

void actuatorHilStep(uint32_t simNowUs) {
    if (!s_hilClockValid) {
        s_hilLastUs     = simNowUs;
        s_hilClockValid = true;
        return;
    }
    uint32_t dtUs = simNowUs - s_hilLastUs;
    s_hilLastUs   = simNowUs;
    if (dtUs == 0 || dtUs > 1000000UL) return;  // skip stale or absurd steps
    hilPlantAdvance(dtUs * 1e-6f);
}
#endif // HIL_MODE && !HIL_REAL_ACTUATOR

// --- ISR + encoder ----------------------------------------------------

#if !defined(HIL_MODE) || defined(HIL_REAL_ACTUATOR)
static void updateState() {
    int a = digitalRead(HALL_A_PIN);
    int b = digitalRead(HALL_B_PIN);
    int encoded = (a << 1) | b;
    int sum = (s_act.lastEncoded << 2) | encoded;

    if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
        s_act.positionCount += ACTUATOR_ENCODER_SIGN;
        s_act.transitionCount++;
    } else if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
        s_act.positionCount -= ACTUATOR_ENCODER_SIGN;
        s_act.transitionCount++;
    }
    s_act.lastEncoded = encoded;
}

static void hallISR() { updateState(); }
#endif // !HIL_MODE || HIL_REAL_ACTUATOR

static long readPositionCount() {
    long c;
    noInterrupts();
    c = s_act.positionCount;
    interrupts();
    return c;
}

static long readTransitionCount() {
    long c;
    noInterrupts();
    c = s_act.transitionCount;
    interrupts();
    return c;
}

// --- Motor helpers ----------------------------------------------------

static void applyMotor(int pwm) {
    pwm = constrain(pwm, -ACTUATOR_PWM_MAX, ACTUATOR_PWM_MAX);
    s_cmd.pwmCmd = pwm;
#if !defined(HIL_MODE) || defined(HIL_REAL_ACTUATOR)
    int raw = pwm * ACTUATOR_EXTEND_SIGN;  // firmware +pwm = extend
    if (raw > 0) {
        analogWrite(ACTUATOR_IN1_PIN, raw);
        analogWrite(ACTUATOR_IN2_PIN, 0);
    } else if (raw < 0) {
        analogWrite(ACTUATOR_IN1_PIN, 0);
        analogWrite(ACTUATOR_IN2_PIN, -raw);
    } else {
        analogWrite(ACTUATOR_IN1_PIN, 0);
        analogWrite(ACTUATOR_IN2_PIN, 0);
    }
#endif
}

static void stopMotor() {
    applyMotor(0);
    s_cmd.dutyPercent = 0.0f;
}

static int dutyToPWM(float duty) {
    if (fabsf(duty) < 1e-3f) return 0;

    int pwm = map((int)fabsf(duty), 0, 100, 0, ACTUATOR_PWM_MAX);

    if (duty > 0.0f && pwm < ACTUATOR_PWM_MIN_EXTEND)   pwm = ACTUATOR_PWM_MIN_EXTEND;
    if (duty < 0.0f && pwm < ACTUATOR_PWM_MIN_RETRACT)  pwm = ACTUATOR_PWM_MIN_RETRACT;

    return duty > 0.0f ? pwm : -pwm;
}

static void zeroPosition() {
    noInterrupts();
    s_act.positionCount  = 0;
    s_act.transitionCount = 0;
#if defined(HIL_MODE) && !defined(HIL_REAL_ACTUATOR)
    s_hilZeroCm = s_hilPosCm;
#else
    s_act.lastEncoded    = (digitalRead(HALL_A_PIN) << 1) | digitalRead(HALL_B_PIN);
#endif
    interrupts();
    s_cmd.integral    = 0.0f;
    s_cmd.dutyPercent = 0.0f;
}

// --- Public API -------------------------------------------------------

void actuatorInit() {
#if !defined(HIL_MODE) || defined(HIL_REAL_ACTUATOR)
    //pinMode(ACTUATOR_NSLEEP_PIN, OUTPUT);
    //digitalWrite(ACTUATOR_NSLEEP_PIN, HIGH);
    delay(5);

    pinMode(ACTUATOR_IN1_PIN, OUTPUT);
    pinMode(ACTUATOR_IN2_PIN, OUTPUT);
    analogWriteFrequency(ACTUATOR_IN1_PIN, ACTUATOR_PWM_FREQ_HZ);
    analogWriteFrequency(ACTUATOR_IN2_PIN, ACTUATOR_PWM_FREQ_HZ);
    stopMotor();

    //pinMode(ACTUATOR_FAULT_PIN, INPUT_PULLUP);
    pinMode(HALL_A_PIN,         INPUT_PULLUP);
    pinMode(HALL_B_PIN,         INPUT_PULLUP);

    s_act.lastEncoded = (digitalRead(HALL_A_PIN) << 1) | digitalRead(HALL_B_PIN);

    attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallISR, CHANGE);
#else  // HIL_MODE && !HIL_REAL_ACTUATOR
    stopMotor();
#endif

    zeroPosition();
}

void actuatorSetTarget(float targetCm) {
    s_cmd.targetCm = targetCm;
}

void actuatorSetEnabled(bool enabled) {
    s_act.enableActuator = enabled;
}

void actuatorUpdatePID() {
    if (!s_act.enableActuator) {
        stopMotor();
        s_cmd.integral = 0.0f;
        return;
    }

    float target = constrain(s_cmd.targetCm, ACTUATOR_MIN_POSITION_CM, ACTUATOR_MAX_POSITION_CM);
    float actual = actuatorGetPositionCm();
    float error  = target - actual;

    s_cmd.dutyPercent = ACTUATOR_KP * error + ACTUATOR_KI * s_cmd.integral;

    // Anti-windup: only integrate when output is not saturated
    float sat = constrain(s_cmd.dutyPercent, ACTUATOR_DUTY_MIN, ACTUATOR_DUTY_MAX);
    if (sat == s_cmd.dutyPercent) {
        s_cmd.integral += error * ACTUATOR_CONTROL_DT_S;
    }

    if (fabsf(error) < ACTUATOR_POSITION_TOLERANCE_CM) {
        stopMotor();
        s_cmd.integral = 0.0f;
        return;
    }

    s_cmd.pwmCmd = dutyToPWM(s_cmd.dutyPercent);
    applyMotor(s_cmd.pwmCmd);
}

float actuatorGetPositionCm() {
    return readPositionCount() * ACTUATOR_CM_PER_TRANSITION;
}

float actuatorGetDutyPercent() {
    return s_cmd.dutyPercent;
}

bool actuatorIsHomed() {
    return s_act.homed;
}

void actuatorHome() {
    s_act.homingActive   = true;
    s_act.enableActuator = true;

    uint32_t startTime  = millis();
    long     lastCount  = readPositionCount();
    uint32_t lastMoveMs = startTime;

    applyMotor(ACTUATOR_HOMING_PWM);  // retract toward hard stop

    while (true) {
        delay(ACTUATOR_HOMING_POLL_MS);

#if defined(HIL_MODE) && !defined(HIL_REAL_ACTUATOR)
        actuatorHilStepDt(ACTUATOR_HOMING_POLL_MS * 1e-3f);
#endif

        if ((uint32_t)(millis() - startTime) > ACTUATOR_HOMING_TIMEOUT_MS) {
            stopMotor();
            s_act.homingActive = false;
            return;
        }

        long count = readPositionCount();
        if (count != lastCount) {
            lastCount  = count;
            lastMoveMs = millis();
        } else if ((uint32_t)(millis() - lastMoveMs) >= ACTUATOR_HOMING_STALL_TIME_MS) {
            stopMotor();
            zeroPosition();
            s_act.homed        = true;
            s_act.homingActive = false;
            return;
        }
    }
}

void actuatorPrintDebug() {
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs < 200) return;
    lastPrintMs = nowMs;

    Serial.print("ACT | H1:");   Serial.print(digitalRead(HALL_A_PIN));
    Serial.print(" H2:");        Serial.print(digitalRead(HALL_B_PIN));
    //Serial.print(" FAULT:");     Serial.print(digitalRead(ACTUATOR_FAULT_PIN));
    Serial.print(" | count:");   Serial.print(readPositionCount());
    Serial.print(" trans:");     Serial.print(readTransitionCount());
    Serial.print(" pos:");       Serial.print(actuatorGetPositionCm(), 4);
    Serial.print(" cm | pwm:");  Serial.print(s_cmd.pwmCmd);
    Serial.print(" duty:");      Serial.print(s_cmd.dutyPercent, 1);
    Serial.print(" | enabled:"); Serial.print(s_act.enableActuator);
    Serial.print(" homed:");     Serial.print(s_act.homed);
    Serial.print(" homing:");    Serial.println(s_act.homingActive);
}
