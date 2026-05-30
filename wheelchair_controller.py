#!/usr/bin/env python3
"""
Muse 2 Mind-Controlled Wheelchair Controller
─────────────────────────────────────────────
Run:  python wheelchair_controller.py [--sim]
Stop: Ctrl+C  │  r = recalibrate

Control scheme (dual-mode):
  Nod forward    (pitch >  7°)   →  Forward
  Lean back      (pitch < -7°)   →  Backward
  Look LEFT      (EOG gaze)      →  Steer left   ← primary steering
  Look RIGHT     (EOG gaze)      →  Steer right  ← primary steering
  Tilt left      (roll < -15°)   →  Steer left   (fallback if scipy missing)
  Tilt right     (roll >  15°)   →  Steer right  (fallback if scipy missing)
  Jaw clench                     →  Emergency STOP
  Double blink                   →  Cycle speed SLOW → MED → FAST
  Head level + eyes centre       →  STOP

Wireless: Muse 2 → BT → PC → BLE → HC-08 → Arduino
Flags:
  --sim   Skip BLE, print commands to terminal (test without Arduino)
"""

import sys, time, signal, threading, queue, argparse, asyncio
from collections import deque

import numpy as np
import bleak
from muselsl.stream import find_muse
from muselsl.muse import Muse

from config import (HC08_ADDRESS, UART_CHAR_UUID,
                    ROLL_THRESHOLD, PITCH_FWD_THRESHOLD, PITCH_BACK_THRESHOLD,
                    EOG_H_THRESHOLD, LOOP_HZ, DISPLAY_HZ)
from artifact_detector import ArtifactDetector

try:
    from scipy.signal import butter, lfilter as _lf
    _SCIPY = True
    # Pre-compute filter coefficients once — butter() is expensive (~512 calls/s otherwise)
    _nyq  = 0.5 * 256
    _EOG_B, _EOG_A = butter(4, [0.5 / _nyq, 10.0 / _nyq], btype='band')
except ImportError:
    _SCIPY = False
    _EOG_B = _EOG_A = None

# ── CLI ───────────────────────────────────────────────────────────────────────
_ap = argparse.ArgumentParser(description='Muse 2 Wheelchair Controller')
_ap.add_argument('--sim', action='store_true',
                 help='Simulation mode — skip BLE, print commands only')
args    = _ap.parse_args()
SIM_MODE = args.sim

# ── Speed levels ──────────────────────────────────────────────────────────────
_SPEED_LABELS = ['SLOW', 'MED ', 'FAST']
_SPEED_CMDS   = ['1',    '2',    '3'   ]
_SPEED_DOTS   = ['■ □ □', '■ ■ □', '■ ■ ■']
_SPEED_COLS   = ['\033[93m', '\033[96m', '\033[92m']
_speed_level  = 1   # 0=slow  1=medium  2=fast

# ── IMU state ─────────────────────────────────────────────────────────────────
_pitch = _roll = _raw_pitch = _raw_roll = 0.0
_pitch_offset = _roll_offset = 0.0
_calibrated  = False
_acc_timeout = 0.0

# ── EOG state ─────────────────────────────────────────────────────────────────
_af7_buf       = deque(maxlen=128)   # 0.5 s at 256 Hz
_af8_buf       = deque(maxlen=128)
_eog_direction = 'CENTER'            # 'LEFT' | 'RIGHT' | 'CENTER'

# ── Control state ─────────────────────────────────────────────────────────────
running  = True
last_cmd = None

# ── BLE transport ─────────────────────────────────────────────────────────────
_ble_queue     = queue.SimpleQueue()
_ble_connected = False
_ble_status    = 'SIM MODE — no BLE' if SIM_MODE else 'Not started'


async def _ble_main():
    global _ble_connected, _ble_status
    while running:
        _ble_connected = False
        _ble_status    = 'Connecting to HC-08...'
        try:
            async with bleak.BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                _ble_connected = True
                _ble_status    = f'Connected  {HC08_ADDRESS}'
                # flush stale queue items
                while True:
                    try:    _ble_queue.get_nowait()
                    except queue.Empty: break
                await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                # Resync speed — Arduino resets to MED on power cycle; Python may differ
                await asyncio.sleep(0.05)
                await client.write_gatt_char(
                    UART_CHAR_UUID, _SPEED_CMDS[_speed_level].encode(), response=True)
                while client.is_connected and running:
                    try:
                        raw = _ble_queue.get_nowait()
                        await client.write_gatt_char(
                            UART_CHAR_UUID, raw.encode(), response=True)
                    except queue.Empty:
                        pass
                    await asyncio.sleep(0.02)
        except Exception as e:
            _ble_status = f'Error: {type(e).__name__} — retrying...'
        _ble_connected = False
        if running:
            await asyncio.sleep(2.0)


def _ble_thread():
    asyncio.run(_ble_main())


def send_cmd(cmd):
    """Send a directional command, deduplicated."""
    global last_cmd
    if cmd == last_cmd:
        return
    last_cmd = cmd
    if SIM_MODE:
        return
    _ble_queue.put(cmd)


def send_raw(raw):
    """Send a one-off command (e.g. speed change) without deduplication."""
    if SIM_MODE:
        return
    _ble_queue.put(raw)


# ── EOG detection ─────────────────────────────────────────────────────────────
def _bandpass(arr):
    return _lf(_EOG_B, _EOG_A, arr)


def _update_eog():
    """Compute horizontal gaze direction from AF7/AF8 EOG asymmetry."""
    global _eog_direction
    if not _SCIPY or len(_af7_buf) < 64:
        return
    af7   = _bandpass(np.array(_af7_buf))
    af8   = _bandpass(np.array(_af8_buf))
    horiz = float(np.mean(af7 - af8))   # positive → LEFT, negative → RIGHT
    if horiz > EOG_H_THRESHOLD:
        _eog_direction = 'LEFT'
    elif horiz < -EOG_H_THRESHOLD:
        _eog_direction = 'RIGHT'
    else:
        _eog_direction = 'CENTER'


# ── Muse callbacks ────────────────────────────────────────────────────────────
def _cycle_speed():
    global _speed_level
    _speed_level = (_speed_level + 1) % 3
    send_raw(_SPEED_CMDS[_speed_level])


detector = ArtifactDetector(
    on_jaw_clench   = lambda: send_cmd('S'),
    on_double_blink = _cycle_speed,
)


def eeg_callback(data, timestamps):
    # Buffer AF7/AF8 for EOG
    for s in data[1]: _af7_buf.append(float(s))
    for s in data[2]: _af8_buf.append(float(s))
    detector.process(data)
    _update_eog()


def acc_callback(data, timestamps):
    global _pitch, _roll, _raw_pitch, _raw_roll, _acc_timeout
    ax = float(data[0][-1])
    ay = float(data[1][-1])
    az = float(data[2][-1])
    _raw_pitch   = float(np.degrees(np.arctan2(ax, np.sqrt(ay**2 + az**2))))
    _raw_roll    = float(np.degrees(np.arctan2(ay, az)))
    _pitch       = _raw_pitch - _pitch_offset
    _roll        = _raw_roll  - _roll_offset
    _acc_timeout = time.time()


def calibrate():
    global _pitch_offset, _roll_offset, _calibrated
    _pitch_offset = _raw_pitch
    _roll_offset  = _raw_roll
    _calibrated   = True


def key_listener():
    import msvcrt
    while running:
        if msvcrt.kbhit() and msvcrt.getch() == b'r':
            calibrate()
        time.sleep(0.05)


def handle_stop(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT,  handle_stop)
signal.signal(signal.SIGTERM, handle_stop)


# ── Control logic ─────────────────────────────────────────────────────────────
def update_control(has_imu):
    if not has_imu:
        send_cmd('S')
        return

    fwd  = _pitch >  PITCH_FWD_THRESHOLD
    back = _pitch < -PITCH_BACK_THRESHOLD

    # EOG gaze takes priority for lateral input.
    # IMU roll is the fallback when scipy is unavailable or gaze is centred.
    eog_active = _eog_direction != 'CENTER'
    left  = (_eog_direction == 'LEFT')  or (not eog_active and _roll < -ROLL_THRESHOLD)
    right = (_eog_direction == 'RIGHT') or (not eog_active and _roll >  ROLL_THRESHOLD)

    if   fwd  and left:   send_cmd('Q')
    elif fwd  and right:  send_cmd('E')
    elif back and left:   send_cmd('G')
    elif back and right:  send_cmd('H')
    elif left:            send_cmd('L')
    elif right:           send_cmd('R')
    elif fwd:             send_cmd('F')
    elif back:            send_cmd('B')
    else:                 send_cmd('S')


# ── Display ───────────────────────────────────────────────────────────────────
_CMD_LABEL = {
    'F':  '\033[92mFORWARD        ▲\033[0m',
    'B':  '\033[93mBACKWARD       ▼\033[0m',
    'L':  '\033[94mSPIN LEFT      ◄\033[0m',
    'R':  '\033[94mSPIN RIGHT     ►\033[0m',
    'Q':  '\033[96mFWD + LEFT     ◤\033[0m',
    'E':  '\033[96mFWD + RIGHT    ◥\033[0m',
    'G':  '\033[96mBCK + LEFT     ◣\033[0m',
    'H':  '\033[96mBCK + RIGHT    ◢\033[0m',
    'S':  '\033[91mSTOP           ■\033[0m',
    None: '\033[90m---\033[0m',
}
_EOG_COLS = {'LEFT': '\033[96m', 'RIGHT': '\033[93m', 'CENTER': '\033[92m'}

# Width of visible content inside the box (excluding '  │' prefix and '│' suffix)
_W = 62


def _row(label, value_plain, value_col=''):
    """Build a fixed-width box row. value_plain is the uncoloured text for padding."""
    prefix = f'  │  {label}'
    pad    = _W - len(prefix) + 2  # +2 for the leading '  '
    padded = f'{value_plain:<{pad}}'
    coloured = f'{value_col}{padded}\033[0m' if value_col else padded
    return f'  │  {label}{coloured}│'


def print_display():
    pitch_lbl = 'NOD  ↓' if _pitch > 5 else 'LEAN ↑' if _pitch < -5 else 'Level •'
    roll_lbl  = 'TILT ◄' if _roll  < -5 else 'TILT ►' if _roll  >  5 else 'Level •'
    cal_lbl   = '\033[92mcalibrated\033[0m' if _calibrated else '\033[93mnot calibrated\033[0m'
    ble_col   = '\033[92m' if _ble_connected else '\033[91m'

    eog_plain = (f'{_eog_direction:<7}'
                 f'{"[EOG active]" if _SCIPY else "[IMU fallback — install scipy for EOG]"}')
    spd_plain = f'[{_SPEED_DOTS[_speed_level]}]  {_SPEED_LABELS[_speed_level]}  (double-blink to cycle)'

    bc = detector.blink_count
    dc = detector.double_blink_count
    cc = detector.clench_count
    stat_plain = f'Blinks:{bc:<4} Dbl:{dc:<4}(→spd)  Clenches:{cc:<4}(→STOP)'

    sim_line = '  \033[95m[ SIMULATION MODE — BLE disabled ]\033[0m\n' if SIM_MODE else ''

    sys.stdout.write('\033[2J\033[H')
    print(f'{sim_line}'
          f'  ┌─ MUSE 2 WHEELCHAIR CONTROLLER ────────────────────────────────┐\n'
          f'  │  BLE    {ble_col}{_ble_status:<54}\033[0m│\n'
          f'  │  Pitch  {_pitch:+6.1f}°  {pitch_lbl}   (fwd/back ±{PITCH_FWD_THRESHOLD:.0f}°)              │\n'
          f'  │  Roll   {_roll:+6.1f}°  {roll_lbl}   (tilt L/R ±{ROLL_THRESHOLD:.0f}°)            │\n'
          f'  │  Gaze   {_EOG_COLS[_eog_direction]}{eog_plain:<54}\033[0m│\n'
          f'  │  Speed  {_SPEED_COLS[_speed_level]}{spd_plain:<54}\033[0m│\n'
          f'  │                                                                │\n'
          f'  │  Command : {_CMD_LABEL.get(last_cmd, _CMD_LABEL[None]):<52}│\n'
          f'  │                                                                │\n'
          f'  │  {stat_plain:<63}│\n'
          f'  └────────────────────────────────────────────────────────────────┘\n'
          f'  Ctrl+C to quit  │  r = recalibrate ({cal_lbl})')
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global running

    sys.stdout.write('\033[2J\033[H')
    print('  Muse 2 Wheelchair Controller')
    print('  ─────────────────────────────')
    if SIM_MODE:
        print('  [SIM] BLE disabled — commands printed to terminal only')
    if not _SCIPY:
        print('  [WARN] scipy not found — EOG gaze disabled, using IMU roll for steering')
        print('         Install with: pip install scipy')
    print()
    print('  Scanning for Muse 2...')

    found = find_muse(backend='bleak')
    if not found:
        print('  No Muse found. Is it powered on and in pairing mode?')
        return

    address = found['address']
    print(f'  Found: {found.get("name", "Muse")}  ({address})')
    print('  Connecting...')

    muse = Muse(
        address=address,
        callback_acc=acc_callback,
        callback_eeg=eeg_callback,
        backend='bleak',
    )

    if not muse.connect():
        print('  Connection failed.')
        return

    muse.start()
    time.sleep(2.0)
    try:    muse.ask_control()
    except: pass

    if not SIM_MODE:
        print(f'  Starting BLE → HC-08 @ {HC08_ADDRESS}')
        threading.Thread(target=_ble_thread, daemon=True).start()

    threading.Thread(target=key_listener, daemon=True).start()
    sys.stdout.write('\033[2J\033[H')

    t_alive   = time.time()
    t_ctrl    = time.time()
    t_display = 0.0
    errors    = 0
    loop_dt   = 1.0 / LOOP_HZ
    disp_dt   = 1.0 / DISPLAY_HZ

    while running:
        now     = time.time()
        has_imu = bool(_acc_timeout) and (now - _acc_timeout) <= 2.0

        if has_imu and not _calibrated:
            calibrate()

        # ── Muse keep-alive + auto-reconnect ─────────────────────────────────
        if now - t_alive > 5:
            try:
                muse.keep_alive()
                errors = 0
            except Exception:
                errors += 1
                if errors >= 3:
                    send_cmd('S')
                    try:    muse.stop(); muse.disconnect()
                    except: pass
                    time.sleep(2.0)
                    if muse.connect():
                        muse.start(); time.sleep(1.0); errors = 0
                    else:
                        print('\n  Muse reconnect failed — exiting.')
                        break
            t_alive = now

        if now - t_ctrl > 2:
            try:    muse.ask_control()
            except: pass
            t_ctrl = now

        update_control(has_imu)

        if now - t_display >= disp_dt:
            print_display()
            t_display = now

        time.sleep(max(0.0, loop_dt - (time.time() - now)))

    send_cmd('S')
    time.sleep(0.3)
    print('\n  Shutting down...')
    try:    muse.stop(); muse.disconnect()
    except: pass
    print('  Done.')


if __name__ == '__main__':
    main()
