// #include "sensors.h"
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include "Adafruit_BMP5xx.h"
// #include "BMI088.h"
// #include <Adafruit_LIS2MDL.h>
// #include "config.h"


// #if USE_SPI_SENSORS //change for spi later
// Adafruit_BMP5xx bmp;

// // For the common Bolder Flight BMI088 library style:
// Bmi088Accel accel(SPI, BMI_ACC_CS_PIN);
// Bmi088Gyro  gyro(SPI, BMI_GYR_CS_PIN);

// Adafruit_LIS2MDL mag;

// void sensorsInit() {
//   Serial.println("Initializing SPI sensors...");

//   // Make all CS pins outputs and deselect all devices first
//   pinMode(BMP_CS_PIN, OUTPUT);
//   pinMode(LIS_CS_PIN, OUTPUT);
//   pinMode(BMI_ACC_CS_PIN, OUTPUT);
//   pinMode(BMI_GYR_CS_PIN, OUTPUT);

//   digitalWrite(BMP_CS_PIN, HIGH);
//   digitalWrite(LIS_CS_PIN, HIGH);
//   digitalWrite(BMI_ACC_CS_PIN, HIGH);
//   digitalWrite(BMI_GYR_CS_PIN, HIGH);

//   SPI.begin();

//   delay(100);

//   // BMP5xx over SPI
//   if (bmp.begin(BMP_CS_PIN, &SPI)) {
//     Serial.println("BMP SPI ok");
//   } else {
//     Serial.println("BMP SPI NOT found");
//   }

//   // BMI088 accel over SPI
//   if (accel.begin()) {
//     Serial.println("BMI088 Accel SPI ok");
//   } else {
//     Serial.println("BMI088 Accel SPI NOT found");
//   }

//   // BMI088 gyro over SPI
//   if (gyro.begin()) {
//     Serial.println("BMI088 Gyro SPI ok");
//   } else {
//     Serial.println("BMI088 Gyro SPI NOT found");
//   }

//   // LIS2MDL over SPI
//   if (mag.begin_SPI(LIS_CS_PIN, &SPI)) {
//     Serial.println("LIS2MDL Mag SPI ok");
//   } else {
//     Serial.println("LIS2MDL Mag SPI NOT found");
//   }

//   Serial.println("Sensor init done.");
// }

// #else

// Adafruit_BMP5xx bmp;
// Bmi088Accel accel(Wire, 0x18);
// Bmi088Gyro  gyro(Wire, 0x68);
// Adafruit_LIS2MDL mag;

// void sensorsInit() {
//   //SENSOR SETUP, add other later
//   Wire.begin();
//   // Wire.setClock(400000); //check sensor speed stuff 

//   if (bmp.begin(0x46, &Wire) || bmp.begin(0x47, &Wire))
//     Serial.println("BMP ok");
//   else
//     Serial.println("BMP NOT found");

//   if (accel.begin())
//     Serial.println("Accel ok");
//   else
//     Serial.println("Accel NOT found");

//   if (gyro.begin())
//     Serial.println("Gyro ok");
//   else
//     Serial.println("Gyro NOT found");

//   if (mag.begin())
//     Serial.println("Mag ok");
//   else
//     Serial.println("Mag NOT found");
// }

// #endif

// float calibrateBaroBase() {
//   float sum = 0.0f;
//   int count = 0;
//   SensorData temp = {};

//   for (int i = 0; i < BARO_CALIB_SAMPLES; i++) {
//     readBaro(temp);

//     if (temp.baroUpdated) {
//       sum += temp.hpa;
//       count++;
//     }

//     delay(20);
//   }

//   if (count == 0) {
//     Serial.println("Baro calibration failed, using sea level.");
//     return SEALEVEL_HPA;
//   }
//   Serial.println("Baro calibrated to local altitude");
//   return sum / count;
// }


// //might be useful to add safety conditionals

// void readIMU(SensorData& data){
//   //check if need to update to version 1.0.2 to put reading in conditional
//   accel.readSensor();
//   gyro.readSensor();

//   data.ax = accel.getAccelX_mss();
//   data.ay = accel.getAccelY_mss();
//   data.az = accel.getAccelZ_mss();

//   data.gx = gyro.getGyroX_rads();
//   data.gy = gyro.getGyroY_rads();
//   data.gz = gyro.getGyroZ_rads();

//   data.imuTimeUs = micros();
//   data.imuUpdated = true;
// }

// void readBaro(SensorData& data){

//   if (bmp.performReading()) {
//     data.hpa = bmp.pressure;
//     data.tempC = bmp.temperature;
//     data.baroTimeUs = micros();
//     data.baroUpdated = true;
//   } 
//   else data.baroUpdated = false;
// }

// void readMag(SensorData& data) {
//   sensors_event_t event;

//   if (mag.getEvent(&event)) {
//     data.mx = event.magnetic.x;
//     data.my = event.magnetic.y;
//     data.mz = event.magnetic.z;

//     data.magTimeUs = micros();
//     data.magUpdated = true;
//   } 
//   else data.magUpdated = false;
  
// }
