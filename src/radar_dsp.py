import serial
import glob
import sys
import threading
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
if not ports:
    print("[-] No serial port found.")
    sys.exit(1)

SERIAL_PORT = ports[0]
BAUD_RATE = 115200
NUM_SAMPLES = 1024
SPEED_OF_SOUND = 343.0  # m/s

# Effective sample interval: ~18 us ADC + 10 us delay = 28 us (~35.7 kSps)
DT = 28e-6
time_axis = np.arange(NUM_SAMPLES) * DT
distance_axis_cm = (SPEED_OF_SOUND * time_axis / 2.0) * 100.0

shared_samples = np.full(NUM_SAMPLES, 2048.0, dtype=np.float32)
data_lock = threading.Lock()
running = True

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
ser.dtr = True
ser.rts = True
ser.reset_input_buffer()
print(f"[+] Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")

def serial_worker():
    global shared_samples, running
    payload_size = NUM_SAMPLES * 2

    while running:
        try:
            if ser.read(1) == b'\xAA':
                if ser.read(3) == b'\xBB\xCC\xDD':
                    raw_payload = ser.read(payload_size)
                    if len(raw_payload) == payload_size:
                        unpacked = np.frombuffer(raw_payload, dtype=np.uint16).astype(np.float32)
                        if np.all(unpacked <= 4095):
                            with data_lock:
                                shared_samples = unpacked
        except Exception:
            break

t_thread = threading.Thread(target=serial_worker, daemon=True)
t_thread.start()

# UI Setup
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
fig.patch.set_facecolor('#121212')
for ax in (ax1, ax2):
    ax.set_facecolor('#1e1e1e')
    ax.tick_params(colors='white')

# 1. Raw Oscilloscope Trace
line_raw, = ax1.plot(distance_axis_cm, shared_samples, color='cyan', lw=1.0)
ax1.set_title("Live Receiver Ultrasonic Scope Trace", color='white')
ax1.set_ylabel("ADC Counts", color='white')
ax1.set_xlim(0, 150)
ax1.set_ylim(1000, 3100)
ax1.grid(True, linestyle='--', alpha=0.3)

# 2. Envelope & Distance Detection
line_env, = ax2.plot(distance_axis_cm, np.zeros(NUM_SAMPLES), color='lime', lw=1.5, label='Echo Envelope')
target_marker, = ax2.plot([], [], 'ro', markersize=9, label='Detected Obstacle')
ax2.set_title("Target Distance Profile", color='white')
ax2.set_xlabel("Target Distance (cm)", color='white')
ax2.set_ylabel("Echo Amplitude", color='white')
ax2.set_xlim(5, 150)
ax2.set_ylim(0, 800)
ax2.grid(True, linestyle='--', alpha=0.3)
ax2.legend(loc='upper right', facecolor='#2a2a2a', edgecolor='none', labelcolor='white')

def update_plot(frame):
    with data_lock:
        raw_data = np.copy(shared_samples)

    line_raw.set_ydata(raw_data)

    ac_signal = raw_data - np.mean(raw_data)
    analytic_signal = scipy.signal.hilbert(ac_signal)
    envelope = np.abs(analytic_signal)

    line_env.set_ydata(envelope)

    # Ignore direct TX bleed-through (< 12 cm)
    valid_mask = distance_axis_cm >= 12.0
    valid_env = envelope[valid_mask]
    valid_dist = distance_axis_cm[valid_mask]

    if len(valid_env) > 0:
        max_idx = np.argmax(valid_env)
        peak_amp = valid_env[max_idx]
        
        if peak_amp > 80:
            target_marker.set_data([valid_dist[max_idx]], [peak_amp])
            ax2.set_title(f"Target Detected at: {valid_dist[max_idx]:.1f} cm (Signal: {int(peak_amp)})", color='#00ff88')
        else:
            target_marker.set_data([], [])
            ax2.set_title("Searching for Target (Clear Path)", color='white')

    return line_raw, line_env, target_marker

ani = FuncAnimation(fig, update_plot, interval=30, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()

running = False
ser.close()