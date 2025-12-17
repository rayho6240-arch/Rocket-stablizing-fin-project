# Flight-Stabilization-Firmware-for-Arduino-Based-RC-Aircraft
Embedded C++ system implementing real-time IMU fusion, PID control, and servo actuation for roll, pitch, and yaw stabilization on Arduino Uno
# Flight Stabilization Firmware – 5 Servo RC Aircraft

This project implements a real-time flight stabilization system for Arduino-based RC aircraft. It utilizes a 6-DOF/9-DOF IMU to measure attitude (roll, pitch, and yaw) and a PCA9685 16-channel PWM driver to control five servos (two ailerons, two elevators, and one rudder). The firmware offers stabilization, EMI (electromagnetic interference) monitoring, and a modular architecture for future extensions, including altitude hold, heading hold, and telemetry.

---

## Key Features
- **5-Servo Control:** Two ailerons, two elevators, one rudder.  
- **IMU-Based Attitude Stabilization:** Roll, pitch, and yaw control.  
- **PWM via PCA9685:** Precise, jitter-free servo signals.  
- **EMI Monitoring:** Reads analog EMI sensor and warns if interference is high.  
- **Modular C++ Firmware:** Clean separation between IMU, Stabilizer, PID, Servo Driver, and Configuration.  
- **Expandable:** Add altitude hold (BMP280) or heading hold (HMC5883L) with minimal changes.  

---

## Hardware Requirements

| Component       | Example Part                              |
|-----------------|--------------------------------------------|
| Microcontroller | Arduino Uno R3                            |
| IMU Sensor      | MPU6050                                   |
| Servo Driver    | Adafruit PCA9685 16-Channel PWM Driver     |
| Servos          | MG90S micro servos                         |
| Battery         | 2S/3S Li-Po                               |
| EMI Probe       | Inductive probe + RC filter                |

---

## Core Modules

| File | Purpose |
|------|----------|
| **main.cpp** | Initializes modules and main loop |
| **imu.cpp/hpp** | Reads IMU and computes roll, pitch, yaw |
| **stabilizer.cpp/hpp** | Mixes attitude correction to five servo outputs |
| **servo_driver.cpp/hpp** | Controls PCA9685 PWM driver |
| **pid.hpp** | Optional PID controller header |
| **config.hpp** | Central configuration for all parameters |

---

## Future Enhancements
- Add altitude hold (BMP280 barometer).  
- Add heading hold (HMC5883L compass).  
- Implement serial telemetry for flight logs.  
- Auto-tune PID gains.  
- Integrate with Azure / Databricks data pipeline.  

---

## Author
**Alejandro Choapa**  
Simon Fraser University – Computer Engineering  
📧 [alejandro.chop99@gmail.com](mailto:alejandro.chop99@gmail.com)  
🔗 [github.com/Alejandrochop99](https://github.com/Alejandrochop99)
