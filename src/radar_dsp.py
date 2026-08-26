import sys
import glob
import struct
import numpy as np
import serial
import matplotlib.pyplot as plt
from scipy.signal import hilbert
from matplotlib.animation import FuncAnimation

# ==============================================================================
# RADAR & SYSTEM CONSTANTS
# ==============================================================================
BAUD = 2000000
FS = 96000                   # 96 kSPS
TC = 0.020                   # 20 ms chirp duration
F_START = 35000              # 35 kHz
F_END = 45000                # 45 kHz
B = F_END - F_START          # 10 kHz Bandwidth
C = 343.0                    # Speed of sound (m/s)
LAMBDA = C / 40000.0         # Acoustic wavelength at 40 kHz (~8.58 mm)

SAMPLES_PER_CHIRP = int(FS * TC)   # 1,920 samples
NUM_CHIRPS = 16                    # Slow-time integration window
N_FFT_RANGE = 2048                 # High-resolution fast-time FFT
SYNC_HEADER = bytes([0xAA, 0xBB, 0xCC, 0xDD])
BYTES_PER_FRAME = SAMPLES_PER_CHIRP * 2

# ==============================================================================
# AUTOMATIC SERIAL PORT DETECTION
# ==============================================================================
def find_esp32_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print("[ERROR] No ESP32 serial port found! Replug the USB cable.")
        sys.exit(1)
    print(f"[INFO] Found active port: {ports[0]}")
    return ports[0]

PORT = find_esp32_port()

# ==============================================================================
# GENERATE COMPLEX ANALYTIC REFERENCE CHIRP (I/Q)
# ==============================================================================
t = np.linspace(0, TC, SAMPLES_PER_CHIRP, endpoint=False)
phase = 2 * np.pi * (F_START * t + (B / (2 * TC)) * (t ** 2))
# Complex analytic reference: cos(phi) - j*sin(phi) = exp(-j*phi)
ref_chirp_complex = np.exp(-1j * phase)

# Physical distance axis mapped from FFT frequency bins
freq_bins = np.fft.rfftfreq(N_FFT_RANGE, d=1.0 / FS)
full_range_axis = (freq_bins * C * TC) / (2.0 * B)

# Truncate view up to 2.2 meters
MAX_DISPLAY_RANGE = 2.2
valid_idx = np.where((full_range_axis >= 0.15) & (full_range_axis <= MAX_DISPLAY_RANGE))[0]
range_axis = full_range_axis[valid_idx]
N_BINS_DISPLAY = len(range_axis)

max_velocity = LAMBDA / (4 * TC)  # ±0.107 m/s
velocity_axis = np.linspace(-max_velocity, max_velocity, NUM_CHIRPS)

# Buffers for Doppler integration and exponential smoothing
slow_time_matrix = np.zeros((NUM_CHIRPS, SAMPLES_PER_CHIRP), dtype=np.complex64)
prev_raw_frame = np.zeros(SAMPLES_PER_CHIRP, dtype=np.float32)
smooth_profile = np.zeros(N_BINS_DISPLAY, dtype=np.float32)

ser = serial.Serial(PORT, BAUD, timeout=1)

def read_synchronized_frame():
    sync_buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            return None
        sync_buf += b
        if len(sync_buf) > 4:
            sync_buf.pop(0)
        if bytes(sync_buf) == SYNC_HEADER:
            raw_data = ser.read(BYTES_PER_FRAME)
            if len(raw_data) == BYTES_PER_FRAME:
                unpacked = struct.unpack(f'<{SAMPLES_PER_CHIRP}H', raw_data)
                return np.array(unpacked, dtype=np.float32)
            return None

# ==============================================================================
# REAL-TIME VISUALIZATION SETUP
# ==============================================================================
fig, (ax_rd, ax_range) = plt.subplots(1, 2, figsize=(13, 5))

im = ax_rd.imshow(np.zeros((NUM_CHIRPS, N_BINS_DISPLAY)), 
                   extent=[0.15, MAX_DISPLAY_RANGE, -max_velocity, max_velocity],
                   aspect='auto', cmap='inferno', origin='lower', vmin=0, vmax=35)
fig.colorbar(im, ax=ax_rd, label='SNR (dB)')
ax_rd.set_title("2D Range-Doppler Heatmap (I/Q MTI)")
ax_rd.set_xlabel("Distance (meters)")
ax_rd.set_ylabel("Radial Velocity (m/s)")

line_profile, = ax_range.plot(range_axis, np.zeros(N_BINS_DISPLAY), color='cyan', lw=2.0)
target_marker, = ax_range.plot([0], [0], 'ro', markersize=8, label='Tracked Object')
ax_range.set_title("1D Fast-Time Target Profile")
ax_range.set_xlabel("Distance (meters)")
ax_range.set_ylabel("Signal Strength (dB)")
ax_range.set_xlim(0.15, 2.0)
ax_range.set_ylim(0, 45)
ax_range.grid(True, linestyle='--', alpha=0.5)
ax_range.legend(loc='upper right')

alert_text = ax_rd.text(0.05, 0.90, "TRACKING CLEAR", transform=ax_rd.transAxes,
                        color='lime', fontsize=12, fontweight='bold',
                        bbox=dict(boxstyle="round,pad=0.3", fc="black", ec="lime", lw=1.5))

frame_count = 0

# ==============================================================================
# FRAME PROCESSING & ANIMATION LOOP
# ==============================================================================
def update_plot(frame):
    global slow_time_matrix, prev_raw_frame, smooth_profile, frame_count
    
    raw_frame = read_synchronized_frame()
    if raw_frame is None:
        return im, line_profile, target_marker, alert_text

    frame_count += 1
    if frame_count % 30 == 0:
        print(f"[ADC Health] Min: {np.min(raw_frame):.0f} | Max: {np.max(raw_frame):.0f} | Mean: {np.mean(raw_frame):.0f} | P-P: {np.ptp(raw_frame):.0f}")

    # 1. 2-Pulse MTI Clutter Cancellation
    rx_diff = raw_frame - prev_raw_frame
    prev_raw_frame = np.copy(raw_frame)
    
    # 2. Complex Analytic Signal via Hilbert Transform
    rx_analytic = hilbert(rx_diff - np.mean(rx_diff))

    # 3. Complex Homodyne Mixing (Dechirping)
    beat_complex = rx_analytic * ref_chirp_complex

    # Roll Slow-Time Buffer
    slow_time_matrix = np.roll(slow_time_matrix, -1, axis=0)
    slow_time_matrix[-1, :] = beat_complex

    # 4. 2D FFT Windowing
    win_fast = np.hanning(SAMPLES_PER_CHIRP)
    win_slow = np.hanning(NUM_CHIRPS)[:, np.newaxis]
    windowed_matrix = slow_time_matrix * win_fast * win_slow

    # Fast-Time Range FFT
    range_fft = np.fft.rfft(windowed_matrix, n=N_FFT_RANGE, axis=1)[:, valid_idx]
    
    # Slow-Time Doppler FFT
    rd_spectrum = np.fft.fftshift(np.fft.fft(range_fft, axis=0), axes=0)
    
    # Power Calculation & Baseline Noise Subtraction
    power_2d = np.abs(rd_spectrum)
    power_db = 20 * np.log10(power_2d + 1e-4)
    
    noise_floor = np.percentile(power_db, 35)
    clean_power_db = np.clip(power_db - noise_floor, 0, 45)

    im.set_data(clean_power_db)

    # Fast 1D Profile with Exponential Moving Average (EMA smoothing)
    instant_profile = np.max(clean_power_db, axis=0)
    smooth_profile = 0.65 * smooth_profile + 0.35 * instant_profile
    line_profile.set_ydata(smooth_profile)

    # 5. CA-CFAR Peak Detection
    peak_idx = np.argmax(smooth_profile)
    detected_range = range_axis[peak_idx]
    peak_power = smooth_profile[peak_idx]

    # CFAR Threshold: require 6.0 dB above local neighborhood
    local_guard = max(1, peak_idx - 6)
    local_guard_end = min(N_BINS_DISPLAY, peak_idx + 6)
    neighborhood = np.concatenate([smooth_profile[:local_guard], smooth_profile[local_guard_end:]])
    local_cfar_floor = np.median(neighborhood) if len(neighborhood) > 0 else 0

    if (peak_power - local_cfar_floor) > 6.0 and peak_power > 8.0:
        target_marker.set_data([detected_range], [peak_power])
        
        vel_profile = clean_power_db[:, peak_idx]
        peak_vel_idx = np.argmax(vel_profile)
        detected_velocity = velocity_axis[peak_vel_idx]

        if detected_velocity < -0.012:
            ttc = detected_range / abs(detected_velocity)
            if ttc < 2.0:
                alert_text.set_text(f"COLLISION WARNING!\nDist: {detected_range:.2f} m | TTC: {ttc:.1f} s")
                alert_text.set_color('red')
                alert_text.set_bbox(dict(boxstyle="round,pad=0.3", fc="black", ec="red", lw=1.5))
            else:
                alert_text.set_text(f"Approaching\nDist: {detected_range:.2f} m | Speed: {abs(detected_velocity):.2f} m/s")
                alert_text.set_color('yellow')
                alert_text.set_bbox(dict(boxstyle="round,pad=0.3", fc="black", ec="yellow", lw=1.5))
        else:
            alert_text.set_text(f"Target Acquired\nDist: {detected_range:.2f} m")
            alert_text.set_color('cyan')
            alert_text.set_bbox(dict(boxstyle="round,pad=0.3", fc="black", ec="cyan", lw=1.5))
    else:
        target_marker.set_data([], [])
        alert_text.set_text("TRACKING CLEAR")
        alert_text.set_color('lime')
        alert_text.set_bbox(dict(boxstyle="round,pad=0.3", fc="black", ec="lime", lw=1.5))

    return im, line_profile, target_marker, alert_text

ani = FuncAnimation(fig, update_plot, interval=20, blit=False)
plt.tight_layout()
plt.show()