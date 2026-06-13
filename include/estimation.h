#pragma once
#include <cstdint>

#include "types.h"
#include "attitude.h"
#include "kalman.h"
#include "config.h"



class Estimator {
public:
  void begin(float basePressure_hPa);
  void update(SensorData& data);
  StateEstimate getState() const;
  float getKalmanP00() const;
  float getKalmanP11() const;

private:
  AttitudeEstimation attitude;
  VerticalKalman kalman;
  StateEstimate state = {};

  float basePressure_hPa = SEALEVEL_HPA;
  uint32_t lastImuTimeUs = 0;
};

float pressureToAlt(float p_hPa, float baseP_hPa);
float verticalAccel(const SensorData& data, const AttitudeState& q);
float computeTiltDeg(const AttitudeState& q);