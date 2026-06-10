#include "Arduino.h"

#include "config.h"
#include "types.h"
#include "sensors.h"
#include "estimation.h"


float basePressure_hPa;
SensorData data = {};
StateEstimate state = {};
Estimator estimator;



volatile int lastEncoded = 0;

float targetCm = 0.0f;
int pwmCmd = 0;


volatile long positionCount = 0;
volatile long transitionCount = 0;

float errorCm = 0.0f;
float integral = 0.0f;
float dutyPercent = 0.0f;


unsigned long lastControlUs = 0;
unsigned long lastPrintMs = 0;

struct ControlCmd{
    float targetCm; //target in cm
    float pwmCmd; //actuator input
    bool enableActuator;
    uint32_t controlTimeUs;

};

struct ActuatorState {
    float positionCm;
    float lastEncoded;
    long positionCount;
    long transitionCount;
    int motorDirection;
    bool homed;
    bool homingActive;
    uint32_t timeUs;
};
ActuatorState actuatorState={};



void hallISR(ActuatorState& actuatorState); //check later how to write ISR
long getPositionCount();
long getTransitionCount();
float getPositionCm();
void zeroPosition();
void setMotor(int pwm);
void stopMotor();
void updateActuatorPID(float targetPositionCm, bool enableActuator);

void setup()
{

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  //initialize sensors
  sensorsInit();
  basePressure_hPa = calibrateBaroBase();
  Serial.print("Base pressure: ");
  Serial.println(basePressure_hPa);

  //initialize filters
  estimator.begin(basePressure_hPa);

  //initialize actuator sensor and calibration (home and calibration: count number of transitions until fully extended and return)

  pinMode(ACTUATOR_IN1_PIN, OUTPUT);
  pinMode(ACTUATOR_IN2_PIN, OUTPUT);

  analogWriteFrequency(ACTUATOR_IN1_PIN, ACTUATOR_PWM_FREQ_HZ);
  analogWriteFrequency(ACTUATOR_IN2_PIN, ACTUATOR_PWM_FREQ_HZ);

  stopMotor();

  pinMode(HALL_A_PIN, INPUT_PULLUP);
  pinMode(HALL_B_PIN, INPUT_PULLUP);

  int a = digitalRead(HALL_A_PIN);
  int b = digitalRead(HALL_B_PIN);

  lastEncoded = (a << 1) | b;

  attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallISR, CHANGE);

  zeroPosition();

  lastControlUs = micros();
  lastPrintMs = millis();



}

void loop()
{

  readIMU(data);
  readBaro(data);
  readMag(data);

  bool imuBefore  = data.imuUpdated;
  bool baroBefore = data.baroUpdated;
  bool magBefore  = data.magUpdated;  

  estimator.update(data);  
  state = estimator.getState();


  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  // Simple actuator test
  if ((nowMs / 5000) % 2 == 0) {
    targetCm = 5.0f;
  } else {
    targetCm = 0.0f;
  }

  if (nowUs - lastControlUs >= ACTUATOR_CONTROL_PERIOD_US) {
    lastControlUs += ACTUATOR_CONTROL_PERIOD_US;
    updateActuatorPID(targetCm, true);
  }

  // --- IMU ---
  Serial.print("IMU | Acc: ");
  Serial.print(data.ax, 2); Serial.print(", ");
  Serial.print(data.ay, 2); Serial.print(", ");
  Serial.print(data.az, 2);

  Serial.print(" | Gyro: ");
  Serial.print(data.gx, 2); Serial.print(", ");
  Serial.print(data.gy, 2); Serial.print(", ");
  Serial.print(data.gz, 2);
  Serial.println();

  // --- Attitude ---
  Serial.print("ATT | Quat: ");
  Serial.print(state.attitude.q0, 4); Serial.print(", ");
  Serial.print(state.attitude.q1, 4); Serial.print(", ");
  Serial.print(state.attitude.q2, 4); Serial.print(", ");
  Serial.print(state.attitude.q3, 4); Serial.print(", ");
  Serial.print(state.attitude.tiltDeg, 4); 
  Serial.println();

  // --- Barometer ---
  Serial.print("BARO | P: ");
  Serial.print(data.hpa, 2);
  Serial.print(" hPa | T: ");
  Serial.print(data.tempC, 2);
  Serial.println(" C");

  // --- Vertical state ---
  Serial.print("VERT | h: ");
  Serial.print(state.vertical.h, 2);
  Serial.print(" m | v: ");
  Serial.print(state.vertical.v, 2);
  Serial.print(" m/s | a: ");
  Serial.print(state.vertical.a, 2);
  Serial.println(" m/s^2");

  // --- Timestamp ---
  Serial.print("TIME | ");
  Serial.println(state.vertical.timeUs);

  Serial.print("FLAGS | IMU ");
  Serial.print(imuBefore);
  Serial.print("->");
  Serial.print(data.imuUpdated);

  Serial.print(" | BARO ");
  Serial.print(baroBefore);
  Serial.print("->");
  Serial.print(data.baroUpdated);

  Serial.print(" | MAG ");
  Serial.print(magBefore);
  Serial.print("->");
  Serial.println(data.magUpdated);

  Serial.print("ACT | target: ");
  Serial.print(targetCm, 2);
  Serial.print(" cm | pos: ");
  Serial.print(getPositionCm(), 3);
  Serial.print(" cm | err: ");
  Serial.print(errorCm, 3);
  Serial.print(" | duty: ");
  Serial.print(dutyPercent, 1);
  Serial.print(" | pwm: ");
  Serial.print(pwmCmd);
  Serial.print(" | count: ");
  Serial.print(getPositionCount());
  Serial.print(" | transitions: ");
  Serial.println(getTransitionCount());

  Serial.println("========================");
  // delay(2000);
}


void hallISR() {
  int a = digitalRead(HALL_A_PIN);
  int b = digitalRead(HALL_B_PIN);

  int encoded = (a << 1) | b;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    positionCount++;
    transitionCount++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    positionCount--;
    transitionCount++;
  }

  lastEncoded = encoded;
}

long getPositionCount() {
  noInterrupts();
  long c = positionCount;
  interrupts();
  return c;
}

long getTransitionCount() {
  noInterrupts();
  long c = transitionCount;
  interrupts();
  return c;
}

float getPositionCm() {
  return getPositionCount() * ACTUATOR_CM_PER_TRANSITION;
}

void zeroPosition() {
  noInterrupts();

  positionCount = 0;
  transitionCount = 0;

  int a = digitalRead(HALL_A_PIN);
  int b = digitalRead(HALL_B_PIN);
  lastEncoded = (a << 1) | b;

  interrupts();

  integral = 0.0f;
  dutyPercent = 0.0f;
}

void setMotor(int pwm) {
  pwm = constrain(pwm, -ACTUATOR_PWM_MAX, ACTUATOR_PWM_MAX);
  pwmCmd = pwm;

  if (pwm > 0) {
    analogWrite(ACTUATOR_IN1_PIN, pwm);
    analogWrite(ACTUATOR_IN2_PIN, 0);
  }
  else if (pwm < 0) {
    analogWrite(ACTUATOR_IN1_PIN, 0);
    analogWrite(ACTUATOR_IN2_PIN, -pwm);
  }
  else {
    analogWrite(ACTUATOR_IN1_PIN, 0);
    analogWrite(ACTUATOR_IN2_PIN, 0);
  }
}

void stopMotor() {
  setMotor(0);
  dutyPercent = 0.0f;
}

int dutyToPWM(float duty) {
  if (fabsf(duty) < 1e-3f) return 0;

  int pwm = map((int)fabsf(duty), 0, 100, 0, ACTUATOR_PWM_MAX);

  if (duty > 0.0f && pwm < ACTUATOR_PWM_MIN_EXTEND) {
    pwm = ACTUATOR_PWM_MIN_EXTEND;
  }

  if (duty < 0.0f && pwm < ACTUATOR_PWM_MIN_RETRACT) {
    pwm = ACTUATOR_PWM_MIN_RETRACT;
  }

  return duty > 0.0f ? pwm : -pwm;
}

void updateActuatorPID(float targetPositionCm, bool enableActuator) {
  if (!enableActuator) {
    stopMotor();
    integral = 0.0f;
    return;
  }

  targetCm = constrain(targetPositionCm, ACTUATOR_MIN_POSITION_CM, ACTUATOR_MAX_POSITION_CM);

  float actualCm = getPositionCm();
  errorCm = targetCm - actualCm;

  dutyPercent = ACTUATOR_KP * errorCm + ACTUATOR_KI * integral;

  if (dutyPercent > ACTUATOR_DUTY_MAX) {
    dutyPercent = ACTUATOR_DUTY_MAX;
  }
  else if (dutyPercent < ACTUATOR_DUTY_MIN) {
    dutyPercent = ACTUATOR_DUTY_MIN;
  }
  else {
    integral += errorCm * ACTUATOR_CONTROL_DT_S;
  }

  if (fabsf(errorCm) < ACTUATOR_POSITION_TOLERANCE_CM) {
    stopMotor();
    integral = 0.0f;
    return;
  }

  setMotor(dutyToPWM(dutyPercent));
}

void setMotor(int pwm) {
  pwm = constrain(pwm, -ACTUATOR_PWM_MAX, ACTUATOR_PWM_MAX);
  pwmCmd = pwm;

  if (pwm > 0) {
    analogWrite(ACTUATOR_IN1_PIN, pwm);
    analogWrite(ACTUATOR_IN2_PIN, 0);
  }
  else if (pwm < 0) {
    analogWrite(ACTUATOR_IN1_PIN, 0);
    analogWrite(ACTUATOR_IN2_PIN, -pwm);
  }
  else {
    analogWrite(ACTUATOR_IN1_PIN, 0);
    analogWrite(ACTUATOR_IN2_PIN, 0);
  }
}

void stopMotor() {
  setMotor(0);
  dutyPercent = 0.0f;
}

int dutyToPWM(float duty) {
  if (fabsf(duty) < 1e-3f) return 0;

  int pwm = map((int)fabsf(duty), 0, 100, 0, ACTUATOR_PWM_MAX);

  if (duty > 0.0f && pwm < ACTUATOR_PWM_MIN_EXTEND) {
    pwm = ACTUATOR_PWM_MIN_EXTEND;
  }

  if (duty < 0.0f && pwm < ACTUATOR_PWM_MIN_RETRACT) {
    pwm = ACTUATOR_PWM_MIN_RETRACT;
  }

  return duty > 0.0f ? pwm : -pwm;
}