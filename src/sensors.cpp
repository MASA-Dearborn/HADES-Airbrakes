#include "sensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include "BMI088.h"
#include <Adafruit_LIS2MDL.h>
#include "config.h"


#if USE_SPI_SENSORS //change for spi later
#else
#endif
Adafruit_BMP5xx bmp;
Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro  gyro(Wire, 0x68);
Adafruit_LIS2MDL mag;

void sensorsInit() {
  //SENSOR SETUP, add other later
  Wire.begin();

  if (bmp.begin(0x46, &Wire) || bmp.begin(0x47, &Wire))
    Serial.println("BMP ok");
  else
    Serial.println("BMP NOT found");

  if (accel.begin())
    Serial.println("Accel ok");
  else
    Serial.println("Accel NOT found");

  if (gyro.begin())
    Serial.println("Gyro ok");
  else
    Serial.println("Gyro NOT found");

  if (mag.begin())
    Serial.println("Mag ok");
  else
    Serial.println("Mag NOT found");
}

float calibrateBaroBase() {
  float sum = 0.0f;
  int count = 0;
  SensorData temp = {};

  for (int i = 0; i < BARO_CALIB_SAMPLES; i++) {
    readBaro(temp);

    if (temp.baroUpdated) {
      sum += temp.hpa;
      count++;
    }

    delay(20);
  }

  if (count == 0) {
    Serial.println("Baro calibration failed, using sea level.");
    return SEALEVEL_HPA;
  }
  Serial.println("Baro calibrated to local altitude");
  return sum / count;
}


//might be useful to add safety conditionals

void readIMU(SensorData& data){
  //check if need to update to version 1.0.2 to put reading in conditional
  accel.readSensor();
  gyro.readSensor();

  data.ax = accel.getAccelX_mss();
  data.ay = accel.getAccelY_mss();
  data.az = accel.getAccelZ_mss();

  data.gx = gyro.getGyroX_rads();
  data.gy = gyro.getGyroY_rads();
  data.gz = gyro.getGyroZ_rads();

  data.imuTimeUs = micros();
  data.imuUpdated = true;
}

void readBaro(SensorData& data){

  if (bmp.performReading()) {
    data.hpa = bmp.pressure;
    data.tempC = bmp.temperature;
    data.baroTimeUs = micros();
    data.baroUpdated = true;
  } 
  else data.baroUpdated = false;
}

void readMag(SensorData& data) {
  sensors_event_t event;

  if (mag.getEvent(&event)) {
    data.mx = event.magnetic.x;
    data.my = event.magnetic.y;
    data.mz = event.magnetic.z;

    data.magTimeUs = micros();
    data.magUpdated = true;
  } 
  else data.magUpdated = false;
  
}
