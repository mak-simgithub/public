import serial
import datetime
import csv
import os
import numpy as np
from scipy.io import wavfile
from scipy.interpolate import interp1d
import signal
import sys

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed
BAUDRATE = 1500000
CHANNEL_NAMES = ['acc_x', 'acc_y', 'acc_z', 'mic']
MIN_SAMPLES = 100  # Minimum samples required to process

# === DATA STORAGE ===
timestamps = []
channels = [[] for _ in CHANNEL_NAMES]
ser = None
writer = None

# === PROCESSING FUNCTION ===
def process_and_save():
    if len(timestamps) < MIN_SAMPLES:
        print(f"Only {len(timestamps)} samples collected. Need at least {MIN_SAMPLES} to process.")
        return

    timestamps_np = np.array(timestamps)
    signals_np = [np.array(ch) for ch in channels]

    time_diffs = np.diff(timestamps_np)
    avg_interval_us = np.mean(time_diffs)
    sample_rate = int(1e6 / avg_interval_us)
    print(f"Estimated sample rate: {sample_rate} Hz")

    duration_us = timestamps_np[-1] - timestamps_np[0]
    num_samples = int(duration_us / avg_interval_us)
    uniform_times = np.linspace(timestamps_np[0], timestamps_np[-1], num_samples)

    def normalize(signal):
        signal = signal - np.mean(signal)
        max_val = np.max(np.abs(signal))
        return (signal / max_val * 32767).astype(np.int16)

    normalized_signals = []
    for sig in signals_np:
        interp_func = interp1d(timestamps_np, sig, kind='linear', fill_value="extrapolate")
        interpolated = interp_func(uniform_times)
        normalized_signals.append(normalize(interpolated))

    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    multi_channel = np.stack(normalized_signals, axis=-1)
    wavfile.write(f"audio/multi_channel_{timestamp_str}.wav", sample_rate, multi_channel)

    for i, name in enumerate(CHANNEL_NAMES):
        wavfile.write(f"audio/{name}_{timestamp_str}.wav", sample_rate, normalized_signals[i])

    print("WAV files saved.")

# === SIGNAL HANDLER ===
def signal_handler(sig, frame):
    print("\nRecording stopped. Processing data...")
    if ser and ser.is_open:
        ser.close()
    process_and_save()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === SERIAL SETUP ===
try:
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE)
except serial.SerialException as e:
    print(f"Failed to open serial port: {e}")
    sys.exit(1)

# === CSV SETUP ===
timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"log/log_{timestamp_str}.csv"
print(f"Logging to {csv_filename}... Press Ctrl+C to stop.")

with open(csv_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['timestamp_us'] + CHANNEL_NAMES)

    while True:
        try:
            line = ser.readline().decode().strip()
            print(f"> {line}")  # 👀 Live serial output

            parts = line.split(',')
            if len(parts) == 5:
                timestamp, acc_x, acc_y, acc_z, mic = map(int, parts)
                writer.writerow([timestamp, acc_x, acc_y, acc_z, mic])
                timestamps.append(timestamp)
                for i, val in enumerate([acc_x, acc_y, acc_z, mic]):
                    channels[i].append(val)
            else:
                print(f"⚠Malformed line: {line}")
        except Exception as e:
            print(f"Error reading line: {e}")
