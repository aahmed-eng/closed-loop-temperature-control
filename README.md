# Closed-Loop Temperature Control System

## Overview
This project implements a **real-time closed-loop temperature control system** using an Arduino microcontroller.  
A noisy temperature signal is sampled, filtered, and used in a **PI feedback controller** to regulate a fan via PWM, maintaining the temperature near a desired setpoint.

The system integrates **signal processing, control theory, embedded programming, and data analysis**, reflecting a complete engineering workflow.

---

## System Architecture

**Sensor**
- DHT11 temperature sensor (digital, noisy, slow dynamics)

**Controller**
- Discrete-time PI controller with:
  - fixed sampling period
  - integrator anti-windup
  - actuator saturation handling

**Filtering**
- Exponential Moving Average (EMA) low-pass filter
- Cutoff frequency selected based on sensor noise and sampling rate

**Actuator**
- DC fan driven via L293D motor driver
- PWM speed control

**Analysis**
- Real-time serial logging
- Python-based plotting and performance evaluation

---

## Control Objective
Maintain the measured temperature at a specified setpoint despite:
- sensor noise
- slow thermal dynamics
- external disturbances (ambient temperature changes)

---

## Hardware Components
- Arduino (Uno-compatible)
- DHT11 temperature sensor
- L293D motor driver IC
- DC fan
- External motor power supply
- Breadboard, resistors, wiring

---

## Software Stack
- **Embedded:** Arduino (C/C++)
- **Analysis & Visualisation:** Python (`pyserial`, `matplotlib`)
- **Control:** Discrete PI controller
- **Filtering:** EMA (first-order IIR)

---

## Control & Signal Processing Details

### Sampling
- Fixed sampling period: 1 s (limited by DHT11)
- Deterministic timing for stable discrete control

### Filtering
The raw temperature signal is filtered using an EMA:

y[k] = α·x[k] + (1 − α)·y[k−1]

The filter reduces high-frequency noise while introducing minimal delay, making it suitable for feedback control.

### Control Law
The PI controller is implemented in discrete time:

u[k] = Kp·e[k] + Ki·∑ e[k]·Ts

with:
- actuator saturation (PWM 0–255)
- integrator anti-windup via clamping

---

## Experimental Results
Typical results demonstrate:
- Reduced sensor noise after filtering
- Stable convergence to setpoint
- Effective disturbance rejection
- Controlled actuator behaviour without integrator windup

Plots include:
- Raw vs filtered temperature
- Control output (PWM)
- Error and integrator state over time

---

## How to Run

1. Wire the hardware according to the schematic.
2. Upload the Arduino sketch.
3. Open the Python live plotting script.
4. Adjust setpoint and controller gains via serial commands.
5. Observe system response and tune parameters.

---

## Limitations & Improvements
- DHT11 limits sampling rate and resolution.
- Thermal dynamics are slow, leading to long settling times.

Possible improvements:
- Replace DHT11 with DS18B20 or thermistor
- Add analogue anti-aliasing RC filter
- Extend to PID control
- Implement model-based tuning

---

## Key Learning Outcomes
- Practical implementation of discrete-time feedback control
- Trade-offs between noise filtering and control delay
- Handling actuator saturation and integrator windup
- End-to-end embedded system design and analysis

---

## Author
Ameen Ahmed

First-year Engineering Science, University of Oxford
