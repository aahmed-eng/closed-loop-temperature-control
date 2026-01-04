# live_plot.py
# Real-time serial plotter for the DHT11 + PI fan controller.
# - Reads CSV lines from serial (as emitted by the Arduino sketch).
# - Plots: raw vs filtered temperature, PWM, and error/integrator.
# - On exit (Ctrl-C) automatically saves the entire session to a timestamped CSV file.
#
# Usage:
#  - update SERIAL_PORT to your port (e.g., 'COM3' or '/dev/ttyACM0')
#  - run: python live_plot.py
#  - stop with Ctrl-C; a CSV file will be written in the current directory.

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import collections
import csv
from datetime import datetime

SERIAL_PORT = 'COM3'    # change to your serial port
BAUD = 115200
WINDOW = 120           # seconds to show in plot (rolling window)

# Open serial port
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(2)

# Rolling buffers for live plotting
times = collections.deque()
setpoints = collections.deque()
temps_raw = collections.deque()
temps_filt = collections.deque()
pwms = collections.deque()
errors = collections.deque()
Is = collections.deque()

# Full-session record (for CSV export)
session_rows = []
# Header for CSV: matches Arduino output and adds a relative_time column
csv_header = ['time_ms', 'relative_time_s', 'setpointC', 'temp_raw', 'temp_filt', 'pwm', 'error', 'integrator']

plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(8, 8))
(ax1, ax2, ax3) = axs

start = time.time()
first_time_ms = None

try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        if line.startswith("time_ms"):  # skip header sent by Arduino
            continue
        parts = line.split(',')
        if len(parts) < 7:
            continue

        # Parse fields output from Arduino
        try:
            t_ms = float(parts[0])
            sp = float(parts[1])
            tr = float(parts[2])
            tf = float(parts[3])
            pwm = float(parts[4])
            e = float(parts[5])
            I = float(parts[6])
        except ValueError:
            # Skip malformed lines
            continue

        # Establish first timestamp to compute relative time for CSV
        if first_time_ms is None:
            first_time_ms = t_ms
        relative_time = (t_ms - first_time_ms) / 1000.0

        # Append to full-session record (for CSV on exit)
        session_rows.append([int(t_ms), relative_time, sp, tr, tf, int(round(pwm)), e, I])

        # Append to rolling buffers for live plotting
        now = time.time() - start
        times.append(now)
        setpoints.append(sp)
        temps_raw.append(tr)
        temps_filt.append(tf)
        pwms.append(pwm)
        errors.append(e)
        Is.append(I)

        # Trim the rolling window
        while times and (times[-1] - times[0] > WINDOW):
            times.popleft()
            setpoints.popleft()
            temps_raw.popleft()
            temps_filt.popleft()
            pwms.popleft()
            errors.popleft()
            Is.popleft()

        # Redraw plots
        ax1.clear()
        ax1.plot(times, temps_raw, label='raw')
        ax1.plot(times, temps_filt, label='filtered')
        ax1.plot(times, setpoints, '--', label='setpoint')
        ax1.set_ylabel('Temp (°C)')
        ax1.legend()
        ax1.grid(True)

        ax2.clear()
        ax2.plot(times, pwms, label='PWM')
        ax2.set_ylabel('PWM')
        ax2.grid(True)

        ax3.clear()
        ax3.plot(times, errors, label='error')
        ax3.plot(times, Is, label='integrator')
        ax3.set_ylabel('error / I')
        ax3.set_xlabel('time (s)')
        ax3.legend()
        ax3.grid(True)

        plt.pause(0.01)

except KeyboardInterrupt:
    # Close serial and save session CSV
    ser.close()
    print("Interrupted by user — saving session to CSV...")

    # Choose filename with timestamp
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"temp_pi_fan_log_{ts_str}.csv"

    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            writer.writerows(session_rows)
        print(f"Saved {len(session_rows)} rows to {filename}")
    except Exception as ex:
        print("Failed to save CSV:", ex)

    print("Exited")
