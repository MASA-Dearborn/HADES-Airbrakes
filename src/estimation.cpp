#include <Arduino.h>
#include "estimation.h"
#include "config.h"
#include <math.h>

// Initialize full estimator pipeline
void Estimator::begin(float basePressure) {
    basePressure_hPa = basePressure;

    attitude.begin();
    kalman.begin(0.0f, 0.0f);

    lastImuTimeUs = 0;
}

// Main estimator update.
// Consumes new sensor flags:
//   IMU  -> Madgwick attitude update + Kalman prediction
//   Baro -> Kalman altitude correction
void Estimator::update(SensorData& data) {
    bool verticalUpdated = false; //update altitude only if either predict or update ran
    if (data.imuUpdated) 
    {
        if (lastImuTimeUs == 0) 
        {
            // First IMU sample only initializes timing; no dt available yet
            lastImuTimeUs = data.imuTimeUs;
        }
        else
        {
            float dt = (data.imuTimeUs - lastImuTimeUs) * 1e-6f; //calculate dt

            if (dt > 0.0f && dt < 0.1f) // do not update if f>10Hz IMU update (safety)
            {
                attitude.updateIMU(
                data.gx, data.gy, data.gz,
                -data.ax, -data.ay, -data.az, 
                dt
                ); // axis inverted for madgwick, might be better to change setup or function
                    //later add update magnetometer, no need for fusion in madwick
                state.attitude = attitude.getState();
                state.attitude.tiltDeg = computeTiltDeg(state.attitude);

                // Project body-frame acceleration into world vertical axis,
                // remove gravity, then use it for Kalman prediction.
                float aVert = verticalAccel(data, state.attitude);

                // if (fabsf(aVert) < 0.25f) aVert = 0.0f; // clear drift test, not sure if needed

                kalman.predict(aVert, dt); // kalman prediction same rate as madg

                verticalUpdated = true;
            }
            lastImuTimeUs = data.imuTimeUs;
        }
        data.imuUpdated = false; 

    }

    if (data.baroUpdated) {
        // Convert pressure to altitude relative to base pressure, if failes it'll use sea level, change if necessary
        float hBaro = pressureToAlt(data.hpa, basePressure_hPa);
        kalman.updateBaro(hBaro); //kalman update

        verticalUpdated = true;
        data.baroUpdated = false; 
    }

    if (verticalUpdated) { 
        state.vertical = kalman.getState();
        // Note: controller consumes latest available estimate (prediction or corrected state)
    }
}

// Return latest fused estimate for controller/logging.
StateEstimate Estimator::getState() const {
    return state;
}

// Barometric altitude equation.
// Computes altitude change relative to base pressure:
//   h = 44330 * (1 - (P / P0)^0.1903)
float pressureToAlt(float p_hPa, float baseP_hPa) {
    return 44330.0f * (1.0f - powf(p_hPa / baseP_hPa, 0.1903f));
}

// Rotate measured body-frame acceleration into world vertical direction.
// Then subtract gravity effect so output is net vertical acceleration.
//
// Assumes:
//   data.ax, ay, az are in m/s^2
//   q is body-to-world attitude estimate
float verticalAccel(const SensorData& data, const AttitudeState& q) {
    float ax = data.ax;
    float ay = data.ay;
    float az = data.az;

    float q0 = q.q0;
    float q1 = q.q1;
    float q2 = q.q2;
    float q3 = q.q3;

    float aWorldZ =
        2.0f * (q1*q3 - q0*q2) * ax +
        2.0f * (q0*q1 + q2*q3) * ay +
        (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;

    return aWorldZ + GRAVITY;
}
// assuming q rotates body -> world and body z-axis is rocket vertical axis
float computeTiltDeg(const AttitudeState& q){
    float zw = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);

    zw = constrain(zw, -1.0f, 1.0f);

    return acosf(zw) * 180.0f / M_PI;

}