#include "kalman.h"
#include "config.h"
#include <Arduino.h>

//Initialize state x = [h; v] and covariance P = I
void VerticalKalman::begin(float h0, float v0) {
    h = h0;
    v = v0;
    a = 0.0f; // last measured vertical acceleration

    P00 = 1.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 1.0f;
}

// Prediction step using IMU acceleration
void VerticalKalman::predict(float aVert, float dt) {
    //   h_k+1 = h_k + v*dt + 0.5*a*dt^2
    h = h + v * dt + 0.5f * aVert * dt * dt;
    //   v_k+1 = v_k + a*dt
    v = v + aVert * dt;

    // Powers of dt for process noise (Q)
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    // Process noise covariance Q
    float Q00 = KF_SIGMA_A2 * dt4 / 4.0f;
    float Q01 = KF_SIGMA_A2 * dt3 / 2.0f;
    float Q10 = Q01;
    float Q11 = KF_SIGMA_A2 * dt2;

    // Covariance propagation (non-Josephus form) P = F P F^T + Q:
    float P00_new = P00 + dt * (P10 + P01) + dt2 * P11 + Q00;
    float P01_new = P01 + dt * P11 + Q01;
    float P10_new = P10 + dt * P11 + Q10;
    float P11_new = P11 + Q11;

    // Update covariance a priori
    P00 = P00_new;
    P01 = P01_new;
    P10 = P10_new;
    P11 = P11_new;

    a = aVert;
}

// Measurement update using barometer altitude
void VerticalKalman::updateBaro(float hBaro) {

    // Innovation covariance S = H P H^T + R
    float S = P00 + KF_R;
    if (S < 1e-6f) return; //not divide by 0

    // Kalman gain K = P H^T S^-1
    float K0 = P00 / S;
    float K1 = P10 / S;

    //residual
    float innovation = hBaro - h;

    // State correction
    h = h + K0 * innovation;
    v = v + K1 * innovation;

    // Covariance update a posteriori P = (I - K H) P:
    float P00_old = P00;
    float P01_old = P01;
    float P10_old = P10;
    float P11_old = P11;

    P00 = (1.0f - K0) * P00_old;
    P01 = (1.0f - K0) * P01_old;
    P10 = P10_old - K1 * P00_old;
    P11 = P11_old - K1 * P01_old;
}

VerticalState VerticalKalman::getState() const {
    VerticalState out;
    out.h = h;
    out.v = v;
    out.a = a;
    out.timeUs = micros();
    return out;
}