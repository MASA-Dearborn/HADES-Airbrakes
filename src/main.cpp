#include "Arduino.h"

#include "config.h"
// #include "types.h"
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
    int pwmCmd; //actuator input
    volatile float dutyPercent;
    volatile float integral;
    bool enableActuator;
    uint32_t controlTimeUs;

};

struct ActuatorState {
    float positionCm;
    int lastEncoded;
    volatile long positionCount;
    volatile long transitionCount;
    int motorDirection;
    bool homed;
    bool homingActive;
    bool enableActuator;
    uint32_t timeUs;
};
ActuatorState actuatorState={};
ControlCmd controlCmd = {};




void updateActuatorState(ActuatorState& actuatorState); 
void hallISR() {
  updateActuatorState(actuatorState); //ISR that calls an function with passable paramter
}  
long getPositionCount(const ActuatorState& actuatorState);
long getTransitionCount(const ActuatorState& actuatorState);
float getPositionCm(const ActuatorState& actuatorState);
void zeroPosition(ActuatorState& actuatorState, ControlCmd& controlCmd );
void setMotor(ControlCmd& controlCmd) ;
void stopMotor(ControlCmd& controlCmd);
void updateActuatorPID(ActuatorState& actuatorState, ControlCmd& controlCmd);
float calibrateActuator(ActuatorState& actuatorState);
void homeActuator(ActuatorState& actuatorState);
int dutyToPWM(float duty);


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

  // pinMode(ACTUATOR_NSLEEP_PIN, OUTPUT);
  // digitalWrite(ACTUATOR_NSLEEP_PIN, HIGH);
  // delay(5);

  // pinMode(ACTUATOR_IN1_PIN, OUTPUT);
  // pinMode(ACTUATOR_IN2_PIN, OUTPUT);

  // analogWriteFrequency(ACTUATOR_IN1_PIN, ACTUATOR_PWM_FREQ_HZ);
  // analogWriteFrequency(ACTUATOR_IN2_PIN, ACTUATOR_PWM_FREQ_HZ);

  // stopMotor(controlCmd);

  // pinMode(ACTUATOR_FAULT_PIN, INPUT_PULLUP);

  // pinMode(HALL_A_PIN, INPUT_PULLUP);
  // pinMode(HALL_B_PIN, INPUT_PULLUP);

  // int a = digitalRead(HALL_A_PIN);
  // int b = digitalRead(HALL_B_PIN);

  // actuatorState.lastEncoded = (a << 1) | b;

  // attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallISR, CHANGE);

  // zeroPosition(actuatorState, controlCmd);

  // lastControlUs = micros();
  // lastPrintMs = millis();



}

void printActuatorDebug(
  const ActuatorState& actuatorState,
  const ControlCmd& controlCmd,
  float targetCm,
  float errorCm
) {
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();

  if (nowMs - lastPrintMs < 200) {
    return;
  }
  lastPrintMs = nowMs;

  int hallA = digitalRead(HALL_A_PIN);
  int hallB = digitalRead(HALL_B_PIN);
  int fault = digitalRead(ACTUATOR_FAULT_PIN);

  long posCount = getPositionCount(actuatorState);
  long transCount = getTransitionCount(actuatorState);
  float posCm = getPositionCm(actuatorState);

  Serial.print("ACT | H1:");
  Serial.print(hallA);

  Serial.print(" H2:");
  Serial.print(hallB);

  Serial.print(" FAULT:");
  Serial.print(fault);  // usually 1 = OK, 0 = fault

  Serial.print(" | count:");
  Serial.print(posCount);

  Serial.print(" trans:");
  Serial.print(transCount);

  Serial.print(" pos:");
  Serial.print(posCm, 4);
  Serial.print(" cm");

  Serial.print(" | pwm:");
  Serial.print(controlCmd.pwmCmd);

  Serial.print(" | duty:");
  Serial.print(controlCmd.dutyPercent, 1);

  Serial.print(" | enabled:");
  Serial.print(actuatorState.enableActuator);

  Serial.print(" | homed:");
  Serial.print(actuatorState.homed);

  Serial.print(" | homing:");
  Serial.println(actuatorState.homingActive);
}

// void loop() {
//   actuatorState.enableActuator = true;

//   Serial.println("Extend test");
//   controlCmd.pwmCmd = -120;
//   setMotor(controlCmd);

//   for (int i = 0; i < 50; i++) {
//     printActuatorDebug(actuatorState, controlCmd, 0.0f, 0.0f);
//     delay(200);
//   }

//   Serial.println("Stop");
//   stopMotor(controlCmd);
//   delay(1000);

//   Serial.println("Retract test");
//   controlCmd.pwmCmd = 120;
//   setMotor(controlCmd);

//   for (int i = 0; i < 50; i++) {
//     printActuatorDebug(actuatorState, controlCmd, 0.0f, 0.0f);
//     delay(200);
//   }

//   Serial.println("Stop");
//   stopMotor(controlCmd);
//   delay(2000);
// }

void loop()
{
  data.imuUpdated = false;
  data.baroUpdated = false;
  data.magUpdated = false;

  readIMU(data);
  readBaro(data);
  readMag(data);

  bool imuBefore  = data.imuUpdated;
  bool baroBefore = data.baroUpdated;
  bool magBefore  = data.magUpdated;  

  // estimator.update(data);  
  // state = estimator.getState();


  // unsigned long nowUs = micros();
  // unsigned long nowMs = millis();

  // // Simple actuator test
  // if ((nowMs / 5000) % 2 == 0) {
  //   targetCm = 5.0f;
  // } else {
  //   targetCm = 0.0f;
  // }

  // if (nowUs - lastControlUs >= ACTUATOR_CONTROL_PERIOD_US) {
  //   lastControlUs += ACTUATOR_CONTROL_PERIOD_US;
  //   controlCmd.targetCm = targetCm;
  //   actuatorState.enableActuator = true;
  //   updateActuatorPID(actuatorState, controlCmd);
  // }

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
  // --- Magnetometer ---

  Serial.print("MAG uT | ");
  Serial.print(data.mx, 3); Serial.print(", ");
  Serial.print(data.my, 3); Serial.print(", ");
  Serial.println(data.mz, 3);
  
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

  // Serial.print("ACT | target: ");
  // Serial.print(targetCm, 2);
  // Serial.print(" cm | pos: ");
  // Serial.print(getPositionCm(), 3);
  // Serial.print(" cm | err: ");
  // Serial.print(errorCm, 3);
  // Serial.print(" | duty: ");
  // Serial.print(dutyPercent, 1);
  // Serial.print(" | pwm: ");
  // Serial.print(pwmCmd);
  // Serial.print(" | count: ");
  // Serial.print(getPositionCount());
  // Serial.print(" | transitions: ");
  // Serial.println(getTransitionCount());

  Serial.println("========================");
  delay(2000);
}


void updateActuatorState(ActuatorState& actuatorState) {
  //reentrant function cause it's inside ISR
  //when either pin activates it'll read both pins
  int a = digitalRead(HALL_A_PIN);
  int b = digitalRead(HALL_B_PIN);
  // previous|current
  int encoded = (a << 1) | b; // := ab
  int sum = (actuatorState.lastEncoded << 2) | encoded; //a'b'ab

  // if  a transitions before b, foward 0b1101
  if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100)
  {
    actuatorState.positionCount++;
    actuatorState.transitionCount++;
  }
  // reverse, reverse
  else if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000)
  {
    actuatorState.positionCount--;
    actuatorState.transitionCount++;
  }

  actuatorState.lastEncoded = encoded;

}

long getPositionCount(const ActuatorState& actuatorState) {
  long c;
  noInterrupts();
  c = actuatorState.positionCount;
  interrupts();
  return c;
}

long getTransitionCount(const ActuatorState& actuatorState) {
  long c;
  noInterrupts();
  c = actuatorState.transitionCount;
  interrupts();
  return c;
}

float getPositionCm(const ActuatorState& actuatorState) {
  return getPositionCount(actuatorState) * ACTUATOR_CM_PER_TRANSITION;
}

void zeroPosition(ActuatorState& actuatorState, ControlCmd& controlCmd ) {
  noInterrupts();

  actuatorState.positionCount = 0;
  actuatorState.transitionCount = 0;

  int a = digitalRead(HALL_A_PIN);
  int b = digitalRead(HALL_B_PIN);
  actuatorState.lastEncoded = (a << 1) | b;

  interrupts();

  controlCmd.integral = 0.0f;
  controlCmd.dutyPercent = 0.0f;
}

//set motor movement stuff
void updateActuatorPID(ActuatorState& actuatorState, ControlCmd& controlCmd) {
  //check actuator enabled, if not block and not integrate
  if (!actuatorState.enableActuator) {
    stopMotor(controlCmd);
    controlCmd.integral = 0.0f;
    return;
  } 

  //constrain targetCM
  targetCm = constrain(controlCmd.targetCm, ACTUATOR_MIN_POSITION_CM, ACTUATOR_MAX_POSITION_CM);

  float actualCm = getPositionCm(actuatorState);
  errorCm = targetCm - actualCm;
  //calculate duty cycle
  controlCmd.dutyPercent = ACTUATOR_KP * errorCm + ACTUATOR_KI * controlCmd.integral;

  //ensure it's between bounds
  int sat = constrain(controlCmd.dutyPercent, ACTUATOR_DUTY_MIN, ACTUATOR_DUTY_MAX);
  //only integrate if error is not saturated, imagine increase aggresiveness if input is small
  if (sat == controlCmd.dutyPercent)
  {
    controlCmd.integral += errorCm * ACTUATOR_CONTROL_DT_S;
  }

  //if smaller than required stop
  if (fabsf(errorCm) < ACTUATOR_POSITION_TOLERANCE_CM) {
    stopMotor(controlCmd);
    controlCmd.integral = 0.0f;
    return;
  }

  // convert to hardware specific command
  controlCmd.pwmCmd = dutyToPWM(controlCmd.dutyPercent);
  setMotor(controlCmd);
}

//check direction
void setMotor(ControlCmd& controlCmd) {
  int pwm = controlCmd.pwmCmd;
  pwm = constrain(pwm, -ACTUATOR_PWM_MAX, ACTUATOR_PWM_MAX);
  controlCmd.pwmCmd = pwm;

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

void stopMotor(ControlCmd& controlCmd) {
  controlCmd.pwmCmd = 0;
  setMotor(controlCmd);
  controlCmd.dutyPercent = 0.0f;
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

// write homing function 
// struct ControlCmd{
//     float targetCm; //target in cm
//     int pwmCmd; //actuator input
//     volatile float dutyPercent;
//     volatile float integral;
//     bool enableActuator;
//     uint32_t controlTimeUs;

// };

// struct ActuatorState {
//     float positionCm;
//     int lastEncoded;
//     volatile long positionCount;
//     volatile long transitionCount;
//     int motorDirection;
//     bool homed;
//     bool homingActive;
//     bool enableActuator;
//     uint32_t timeUs;
// };


void homeActuator(ActuatorState& actuatorState, ControlCmd& controlCmd){
  //return to 0, detect 0 using timing and reset transition count
  actuatorState.homingActive=true;
  actuatorState.enableActuator=true;

  uint32_t startTime = millis();
  long lastCount = getPositionCount(actuatorState);
  uint32_t lastMoveTime = millis();

  controlCmd.pwmCmd = ACTUATOR_HOMING_PWM;
  setMotor(controlCmd);

  



  

}

float calibrateActuator(ActuatorState& actuatorState){
  //calculate the number of transitions for full extnt of lin actuator, only for prior calibration

}
