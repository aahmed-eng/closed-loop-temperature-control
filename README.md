# Closed-Loop Temperature Control System

## Overview
This project implements a real-time closed-loop temperature control system using an Arduino microcontroller.  
A noisy temperature signal is sampled, filtered, and used in a PI feedback controller to regulate a fan via PWM, maintaining the temperature near a desired setpoint.

The system integrates signal processing, control theory, embedded programming, and data analysis.

## Requirements

- PySerial
- NumPy
- Matplotlib

Install dependencies with:

```bash
pip install -r requirements.txt
```

## Hardware Components
- Arduino (Uno-compatible)
- DHT11 temperature sensor
- L293D motor driver IC
- DC fan
- External motor power supply
- Breadboard, resistors, wiring

## Control & Signal Processing Details

### Filtering
The raw temperature signal is filtered using an EMA:

y[k] = α·x[k] + (1 − α)·y[k−1]

The filter reduces high-frequency noise while introducing minimal delay, which makes it suitable for feedback control.

### Control Law
The PI controller is implemented in discrete time:

u[k] = Kp·e[k] + Ki·∑ e[k]·Ts

with:
- actuator saturation (PWM 0–255)
- integrator anti-windup via clamping

## Limitations & Improvements
- DHT11 limits sampling rate and resolution.
- Thermal dynamics are slow, leading to long settling times.

Possible improvements:
- Replace DHT11 with DS18B20 or thermistor
- Add analogue anti-aliasing RC filter
- Extend to PID control
- Implement model-based tuning
