#include "Arduino.h"
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include "BMI088.h"
#include <Adafruit_LIS2MDL.h>


#include "config.h"
#include "types.h"
#include "sensors.h"





float pressureToAlt(float p_hPa, float baseP_hPa);
float basePressure_hPa;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  //initialize sensors
  sensorsInit();
  basePressure_hPa = calibrateBaroBase();


}

void loop()
{
  SensorData data = {};

  readIMU(data);
  readBaro(data);
  readMag(data);
  // ================= PRINT =================
  Serial.println("===== SENSOR DATA =====");
  if (data.imuUpdated)
  {
    Serial.print("Accel (m/s^2): ");
    Serial.print(data.ax); Serial.print(", ");
    Serial.print(data.ay); Serial.print(", ");
    Serial.println(data.az); Serial.println(data.imuTimeUs); 

    Serial.print("Gyro (rad/s): ");
    Serial.print(data.gx); Serial.print(", ");
    Serial.print(data.gy); Serial.print(", ");
    Serial.println(data.gz); Serial.println(data.imuTimeUs); 
}
  if (data.magUpdated)
  {
    Serial.print("Mag (uT): ");
    Serial.print(data.mx); Serial.print(", ");
    Serial.print(data.my); Serial.print(", ");
    Serial.println(data.mz); Serial.println(data.magTimeUs); 
  }

  Serial.print("Pressure (hPa): ");
  Serial.print(data.hpa);
  Serial.print("  Temp (C): ");
  Serial.println(data.tempC); Serial.println(data.baroTimeUs); 

  Serial.println("=======================\n");
  delay(1000);
}

float pressureToAlt(float p_hPa, float baseP_hPa){
  return 44330.0f * (1.0f - powf(p_hPa / baseP_hPa, 0.1903f));
}
