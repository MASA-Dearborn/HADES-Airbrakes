#pragma once
#include "types.h"


void sensorsInit();
float calibrateBaroBase();
void readIMU(SensorData& data);
void readBaro(SensorData& data);
void readMag(SensorData& data);