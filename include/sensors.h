#pragma once
#include "types.h"


void sensorsInit();
void readIMU(SensorData& data);
void readBaro(SensorData& data);
void readMag(SensorData& data);