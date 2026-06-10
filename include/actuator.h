#pragma once

void actuatorInit();
void actuatorHome();
void actuatorSetTarget(float targetCm);
void actuatorSetEnabled(bool enabled);
void actuatorUpdatePID();
float actuatorGetPositionCm();
bool actuatorIsHomed();
void actuatorPrintDebug();
