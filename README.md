# HADES Airbrakes – Estimation & Sensor Fusion

## Key Modules

- attitude: Madgwick-based orientation estimation (IMU)
- kalman: Vertical state estimation (h, v, a)
- estimation: Sensor fusion pipeline (IMU + barometer)
- sensors: Hardware interface for IMU, barometer, magnetometer
- types: Shared data structures
- config: System constants and tuning parameters

--------------------------------------------------

## Estimation Overview

### Attitude Estimation
- Quaternion-based orientation
- Uses gyroscope integration + accelerometer correction
- No magnetometer fusion (yaw drifts over time)

### Vertical Estimation
- 2-state Kalman filter:
  x = [h, v]^T

### Prediction:
- Uses IMU-derived vertical acceleration

### Correction:
- Uses barometric altitude

--------------------------------------------------

## To Build & Upload

install PlatformIO CLI beforehand and run

```bash
pio run
pio run --target upload
```

--------------------------------------------------

## Steps

0. Theory, module testing [✓]
1. Sensor Interface [✓] 
    - TODO: add SPI support
2. State Estimation [✓]
3. Actuator Driver/Sensor/Controller
4. Guidance Controller (Simple one is tested, but need to add drag data)
5. State Machine
6. Safety & Inhibit Logic
7. RF telemetry
8. Logging & HIL Testing


--------------------------------------------------
## Documentation:


--------------------------------------------------
## Contributors

Contributors are listed alphabetically. Roles indicate primary areas of contribution.

### MADA-D - Project HADES 2026 – Airbrakes Electrical Team

Marcus Wada – Project Lead  
Andrew Bellinger – Theory & Testing  
Ali Beidoun – CFD & Assembly  
Osman Hadi – PCB Design & Schematics  
Resul Ilmammedov – Hardware Prototyping  
David Richardson – Theory  

### Essential Support & Leadership

Robert Everitt – Electrical Chief Engineer  
Michael Miney – Mechanical Chief Engineer (CFD)  
Chase Sutherlin – Airbrakes Mechanical Design  
Karishma Patnaik - Assistant Professor

