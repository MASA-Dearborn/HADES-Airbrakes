#include "attitude.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

// Initialize attitude quaternion to identity rotation q=[1, 0, 0, 0]
void AttitudeEstimation::begin() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

// Madgwick IMU update using gyro + accelerometer only.
// Gyro integrates angular velocity, while accel corrects roll/pitch
// by aligning estimated gravity with measured gravity.
void AttitudeEstimation::updateIMU(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   float dt) {
    if (dt <= 0.0f) return;
    
    // Normalize accelerometer measurement so it represents direction of gravity.
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    //Compute quaternion terms

    float twoQ0 = 2.0f * q0;
    float twoQ1 = 2.0f * q1;
    float twoQ2 = 2.0f * q2;
    float twoQ3 = 2.0f * q3;
    float fourQ0 = 4.0f * q0;
    float fourQ1 = 4.0f * q1;
    float fourQ2 = 4.0f * q2;
    float eightQ1 = 8.0f * q1;
    float eightQ2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Gradient descent correction step: 
    // f(q, a) is the error term: min ||f(q, a)^2||
    // df = J^T(q_t-1)f(q_t-1, a_t)

    float s0 = fourQ0 * q2q2 + twoQ2 * ax + fourQ0 * q1q1 - twoQ1 * ay;

    float s1 = fourQ1 * q3q3 - twoQ3 * ax
            + 4.0f * q0q0 * q1 - twoQ0 * ay
            - fourQ1
            + eightQ1 * q1q1
            + eightQ1 * q2q2
            + fourQ1 * az;

    float s2 = 4.0f * q0q0 * q2 + twoQ0 * ax
            + fourQ2 * q3q3 - twoQ3 * ay
            - fourQ2
            + eightQ2 * q1q1
            + eightQ2 * q2q2
            + fourQ2 * az;

    float s3 = 4.0f * q1q1 * q3 - twoQ1 * ax
            + 4.0f * q2q2 * q3 - twoQ2 * ay;
    
    //Normalize for consistant beta gain
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 1e-6f) {
        s0 /= norm;
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
    }

    // Quaternion derivative:
    // q_dot = 0.5 * q ⊗ omega - beta * gradient
    // First term integrates gyro; second term corrects drift using accel
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - MADGWICK_BETA * s0;
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - MADGWICK_BETA * s1;
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - MADGWICK_BETA * s2;
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - MADGWICK_BETA * s3;

    // Euler integration of quaternion state.
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Re-normalize quaternion
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-6f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
}

AttitudeState AttitudeEstimation::getState() const {
    AttitudeState out;
    out.q0 = q0;
    out.q1 = q1;
    out.q2 = q2;
    out.q3 = q3;
    out.timeUs = micros();
    return out;
}