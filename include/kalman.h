#pragma once
#include "types.h"

class VerticalKalman {
public:
  void begin(float h0 = 0.0f, float v0 = 0.0f);
  void predict(float aVert, float dt);
  void updateBaro(float hBaro);
  VerticalState getState() const;
  float getP00() const { return P00; }
  float getP11() const { return P11; }

private:
  float h = 0.0f;
  float v = 0.0f;
  float a = 0.0f;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};