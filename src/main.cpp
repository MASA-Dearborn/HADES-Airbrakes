#include "Arduino.h"

#include "config.h"
#include "types.h"
#include "sensors.h"
#include "estimation.h"


float basePressure_hPa;
SensorData data = {};
StateEstimate state = {};
AttitudeEstimation madgwick;
VerticalKalman kalman;
Estimator estimator;

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

  // madgwick.begin();
  // kalman.begin(0.0f, 0.0f);


}

void loop()
{
  // static uint32_t lastImuTimeUs = 0; //defined in Estimator

  readIMU(data);
  readBaro(data);
  readMag(data);

  bool imuBefore  = data.imuUpdated;
  bool baroBefore = data.baroUpdated;
  bool magBefore  = data.magUpdated;  

  estimator.update(data);  
  state = estimator.getState();

  
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
  Serial.print(state.attitude.q3, 4);
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

  Serial.println("========================");
  // delay(2000);
}

