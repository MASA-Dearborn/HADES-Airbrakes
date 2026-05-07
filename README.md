HADES Airbrakes – Estimation & Sensor Fusion

Key Modules

- attitude: Madgwick-based orientation estimation (IMU)
- kalman: Vertical state estimation (h, v, a)
- estimation: Sensor fusion pipeline (IMU + barometer)
- sensors: Hardware interface for IMU, barometer, magnetometer
- types: Shared data structures
- config: System constants and tuning parameters

--------------------------------------------------

Estimation Overview

Attitude Estimation
- Quaternion-based orientation
- Uses gyroscope integration + accelerometer correction
- No magnetometer fusion (yaw drifts over time)

Vertical Estimation
- 2-state Kalman filter:
  x = [h, v]^T

Prediction:
- Uses IMU-derived vertical acceleration

Correction:
- Uses barometric altitude

--------------------------------------------------

To Build & Upload
install PlatformIO CLI beforehand and run

pio run
pio run --target upload

--------------------------------------------------