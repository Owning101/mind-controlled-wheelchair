#!/usr/bin/env python3
"""
Muse 2 EEG + IMU Viewer
Run: C:\\Users\\Admin\\OneDrive\\Desktop\\eeg_env\\Scripts\\python.exe muse_viewer.py
Stop: Ctrl+C
"""
import sys
import time
import signal
import json
from collections import deque
import numpy as np
from muselsl.stream import find_muse
from muselsl.muse import Muse
from artifact_detector import ArtifactDetector

CHANNEL_NAMES = ['TP9  Left ear      ', 'AF7  Left forehead ',
                 'AF8  Right forehead', 'TP10 Right ear     ']
HSI_TAG = {1: '\033[92mGood\033[0m', 2: '\033[93mOk  \033[0m', 4: '\033[91mNone\033[0m'}

# ── EEG state ─────────────────────────────────────────────────────────────────
latest_eeg   = None
hsi          = [4, 4, 4, 4]
running      = True
data_timeout = 0.0
_noise       = [deque(maxlen=12) for _ in range(4)]

detector = ArtifactDetector()

# ── IMU state ─────────────────────────────────────────────────────────────────
_acc_timeout  = 0.0
_gyro_timeout = 0.0
_pitch = 0.0
_roll  = 0.0
_gx = _gy = _gz = 0.0

IMU_DT = 1.0 / 52


# ── callbacks ─────────────────────────────────────────────────────────────────
def eeg_callback(data, timestamps):
    global latest_eeg, data_timeout
    latest_eeg   = [float(np.mean(ch)) for ch in data]
    data_timeout = time.time()
    for i in range(min(4, len(data))):
        _noise[i].append(float(np.std(data[i])))
    detector.process(data)


def acc_callback(data, timestamps):
    global _pitch, _roll, _acc_timeout
    ax = float(data[0][-1])
    ay = float(data[1][-1])
    az = float(data[2][-1])
    _pitch = float(np.degrees(np.arctan2(ax, np.sqrt(ay**2 + az**2))))
    _roll  = float(np.degrees(np.arctan2(ay, az)))
    _acc_timeout = time.time()


def gyro_callback(data, timestamps):
    global _gx, _gy, _gz, _gyro_timeout
    _gx           = float(data[0][-1])
    _gy           = float(data[1][-1])
    _gz           = float(data[2][-1])
    _gyro_timeout = time.time()


def control_callback(msg):
    try:
        d = json.loads(msg.replace("'", '"'))
        keys = ['tp9', 'af7', 'af8', 'tp10']
        if all(k in d for k in keys):
            vals = [int(d[k]) for k in keys]
            if all(v in (1, 2, 4) for v in vals):
                hsi[:] = vals
    except Exception:
        pass


# ── helpers ───────────────────────────────────────────────────────────────────
def signal_tag(ch):
    buf = _noise[ch]
    if len(buf) < 4:
        return '\033[90m--- \033[0m'
    avg = float(np.mean(buf))
    val = latest_eeg[ch] if latest_eeg else 0.0
    if avg < 1.0:  return '\033[90mFlat\033[0m'
    if avg < 20:
        if abs(val) > 300:
            return '\033[93mDC  \033[0m'
        return '\033[92mGood\033[0m'
    if avg < 60:   return '\033[93mOk  \033[0m'
    return             '\033[91mPoor\033[0m'


def _bar(value, maxval=90, width=20):
    frac   = min(abs(value) / maxval, 1.0)
    filled = int(frac * width)
    bar    = '█' * filled + '░' * (width - filled)
    return bar


def _pitch_label(deg):
    if   deg >  5: return '\033[93mNOD DOWN   ↓\033[0m'
    elif deg < -5: return '\033[93mNOD UP     ↑\033[0m'
    else:          return '\033[90mLevel      •\033[0m'


def _roll_label(deg):
    if   deg >  5: return '\033[93mTILT RIGHT →\033[0m'
    elif deg < -5: return '\033[93mTILT LEFT  ←\033[0m'
    else:          return '\033[90mLevel      •\033[0m'


def handle_stop(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT,  handle_stop)
signal.signal(signal.SIGTERM, handle_stop)


# ── display ───────────────────────────────────────────────────────────────────
def print_display():
    now      = time.time()
    has_eeg  = latest_eeg is not None and data_timeout and (now - data_timeout) <= 2.0
    has_acc  = _acc_timeout  and (now - _acc_timeout)  <= 2.0
    has_gyro = _gyro_timeout and (now - _gyro_timeout) <= 2.0
    hsi_live = any(v != 4 for v in hsi)

    sys.stdout.write('\033[2J\033[H')

    # ── EEG section ──
    print(f'  ┌─ EEG  {time.strftime("%H:%M:%S")} ──────────────────────────────────────┐')
    for i, name in enumerate(CHANNEL_NAMES):
        val     = f'{latest_eeg[i]:+9.2f}' if has_eeg else '      ---'
        quality = f'HSI: {HSI_TAG.get(hsi[i], "?")}' if hsi_live else f'sig: {signal_tag(i)}'
        print(f'  │  {name}  {val} µV  [{quality}]')
    if has_eeg and latest_eeg and len(latest_eeg) > 4:
        print(f'  │  AUX                        {latest_eeg[4]:+9.2f} µV')
    print(f'  │')
    bc = detector.blink_count
    cc = detector.clench_count

    cal_tag = '\033[92mcalibrated\033[0m' if detector.baseline_ready else '\033[93mcalibrating…\033[0m'
    blink_str  = (f'\033[96mBLINK \033[0m  count: {bc}   last: {list(detector.blink_log)[-1]}'
                  if bc else f'Blink   [{cal_tag}]  waiting for blink…')
    clench_str = (f'\033[95mCLENCH\033[0m  count: {cc}   last: {list(detector.clench_log)[-1]}'
                  if cc else 'Clench  waiting for jaw clench…')

    if latest_eeg is not None:
        af7_peak  = f'{max(abs(v) for v in _noise[1]):.0f}' if _noise[1] else '---'
        af8_peak  = f'{max(abs(v) for v in _noise[2]):.0f}' if _noise[2] else '---'
        tp9_rms   = f'{float(np.sqrt(np.mean(np.square([latest_eeg[0]])))):.0f}' if latest_eeg else '---'
        tp10_rms  = f'{float(np.sqrt(np.mean(np.square([latest_eeg[3]])))):.0f}' if latest_eeg else '---'
        debug_str = f'dbg  AF7={af7_peak}µV AF8={af8_peak}µV  TP9rms={tp9_rms}µV TP10rms={tp10_rms}µV'
    else:
        debug_str = 'dbg  no EEG data yet'

    print(f'  │  {blink_str}')
    print(f'  │  {clench_str}')
    print(f'  │  \033[90m{debug_str}\033[0m')
    print(f'  └─────────────────────────────────────────────────────────────┘')

    print()

    # ── IMU section ──
    print(f'  ┌─ HEAD ORIENTATION ──────────────────────────────────────────┐')
    if has_acc:
        pl = _pitch_label(_pitch)
        rl = _roll_label(_roll)
        pb = _bar(_pitch)
        rb = _bar(_roll)
        psign = '+' if _pitch >= 0 else '-'
        rsign = '+' if _roll  >= 0 else '-'
        print(f'  │  Pitch (fwd/back)  {psign}{abs(_pitch):5.1f}°  [{pb}]  {pl}')
        print(f'  │  Roll  (L/R tilt)  {rsign}{abs(_roll):5.1f}°  [{rb}]  {rl}')
    else:
        print(f'  │  Pitch (fwd/back)      ---°')
        print(f'  │  Roll  (L/R tilt)      ---°')

    print(f'  │')

    if has_gyro:
        def rate_str(v):
            arrow = '↑' if v > 5 else ('↓' if v < -5 else '·')
            return f'{v:+7.1f} °/s {arrow}'
        print(f'  │  Gyro X (pitch rate)  {rate_str(_gx)}')
        print(f'  │  Gyro Y (roll  rate)  {rate_str(_gy)}')
        print(f'  │  Gyro Z (yaw   rate)  {rate_str(_gz)}')
    else:
        print(f'  │  Gyro  ---')

    print(f'  └─────────────────────────────────────────────────────────────┘')

    sys.stdout.flush()


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    global running

    print('Scanning for Muse 2 (Bluetooth)...')
    found = find_muse(backend='bleak')
    if not found:
        print('No Muse device found.')
        return

    address = found['address']
    name    = found.get('name', 'Muse')
    print(f'Found: {name}  ({address})')
    print('Connecting...')

    muse = Muse(address=address,
                callback_eeg=eeg_callback,
                callback_control=control_callback,
                callback_acc=acc_callback,
                callback_gyro=gyro_callback,
                backend='bleak')

    if not muse.connect():
        print('Connection failed.')
        return

    muse.start()
    time.sleep(1.0)
    try:    muse.ask_control()
    except: pass

    print('Connected. Press Ctrl+C to stop.\n')

    t_alive = time.time()
    t_ctrl  = time.time()
    errors  = 0

    while running:
        now = time.time()

        if now - t_alive > 5:
            try:
                muse.keep_alive()
                errors = 0
            except Exception:
                errors += 1
                if errors >= 3:
                    print('\nReconnecting...')
                    try:    muse.stop(); muse.disconnect()
                    except: pass
                    time.sleep(2.0)
                    if not muse.connect():
                        print('Reconnect failed.'); running = False; break
                    muse.start(); time.sleep(1.0); errors = 0
            t_alive = now

        if now - t_ctrl > 2:
            try:    muse.ask_control()
            except: pass
            t_ctrl = now

        print_display()

        time.sleep(0.02)

    print('\nDisconnecting...')
    try:    muse.stop(); muse.disconnect()
    except: pass
    print('Done.')


if __name__ == '__main__':
    main()