#!/usr/bin/env python3
"""
brainflow_car_controller.py  —  Hybrid BCI wheelchair / RC-car controller
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Best-of-both-worlds design
  · muselsl + bleak    — Muse 2 connection (gives EEG *and* IMU)
  · alberthodo-style   — scipy bandpass filters for reliable double-blink
                         and jaw-clench detection (from BCI repo signal logic)
  · Gyro / accel IMU  — pitch + roll head-tilt steering (from existing project)

Note on BrainFlow: BrainFlow 5.x for Muse only streams EEG channels — it
does not expose the accelerometer or gyro.  muselsl is therefore used for
the hardware layer so we get *both* EEG (for BCI detection) and IMU (for
tilt steering) in a single connection.  The signal-processing algorithms are
ported directly from alberthodo's BCI repo.

Run:   .venv312\Scripts\python.exe brainflow_car_controller.py [flags]
Flags:
  --sim              No hardware — display only
  --com  COM9        Serial / USB-CDC to Arduino instead of BLE HC-08
  --mac  AA:BB:..    Skip Bluetooth scan, use this Muse MAC directly

Stop:  Ctrl+C   |   r = recalibrate head position

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Control scheme
  Jaw clench   (EMG burst TP9/TP10 > 3× adaptive baseline) → STOP  [priority]
  Double blink (2 EOG peaks AF7/AF8 within 0.2–1.2 s)      → cycle speed
  Nod forward  (pitch > +7°)                                → F
  Lean back    (pitch < –7°)                                → B
  Tilt left    (roll  < –15°)                               → L  (+ diagonals)
  Tilt right   (roll  > +15°)                               → R  (+ diagonals)
  Head level, no gesture                                    → S

Output commands
  F B L R Q E G H S   — direction
  1 2 3                — speed (SLOW / MED / FAST)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import sys, time, signal, threading, queue, argparse, asyncio, math, logging
from collections import deque

import numpy as np
from scipy.signal import butter, lfilter

import bleak
from muselsl.stream import find_muse
from muselsl.muse import Muse

try:
    import serial as _pyserial
    _HAS_SERIAL = True
except ImportError:
    _HAS_SERIAL = False

from config import (HC08_ADDRESS, UART_CHAR_UUID,
                    ROLL_THRESHOLD, PITCH_FWD_THRESHOLD, PITCH_BACK_THRESHOLD,
                    LOOP_HZ, DISPLAY_HZ)

# ── CLI ───────────────────────────────────────────────────────────────────────
_ap = argparse.ArgumentParser(description='BrainFlow-style Hybrid BCI Controller')
_ap.add_argument('--sim', action='store_true', help='No hardware — display only')
_ap.add_argument('--com', default='',          help='Serial port, e.g. COM9')
_ap.add_argument('--mac', default='',          help='Muse MAC — skip scan')
args = _ap.parse_args()

logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

# ── Speed ─────────────────────────────────────────────────────────────────────
_SPEED_LABELS = ['SLOW', 'MED ', 'FAST']
_SPEED_CMDS   = ['1',    '2',    '3'   ]
_SPEED_DOTS   = ['■ □ □', '■ ■ □', '■ ■ ■']
_SPEED_COLS   = ['\033[93m', '\033[96m', '\033[92m']
_speed_level  = 1   # 0=slow 1=med 2=fast

# ── Shared state ──────────────────────────────────────────────────────────────
_running     = True
_last_cmd    = None

# IMU
_pitch       = 0.0
_roll        = 0.0
_raw_pitch   = 0.0
_raw_roll    = 0.0
_pitch_off   = 0.0
_roll_off    = 0.0
_calibrated  = False
_imu_ts      = 0.0

# BCI event flags — set by SignalProcessor, consumed once per control tick
_jaw_pending    = False
_dblink_pending = False

# Display stats
_blink_count  = 0
_dblink_count = 0
_jaw_count    = 0

# Signal quality (HSI from Muse artifact packets: 1=good 2=ok 4=poor)
_hsi = [4, 4, 4, 4]

# ── Output ────────────────────────────────────────────────────────────────────
_ble_queue     = queue.SimpleQueue()
_ble_connected = False
_ble_status    = 'SIM MODE' if args.sim else 'Not started'
_serial_port   = None


def send_cmd(cmd: str) -> None:
    global _last_cmd
    if cmd == _last_cmd:
        return
    _last_cmd = cmd
    _dispatch(cmd)


def send_raw(raw: str) -> None:
    _dispatch(raw)


def _dispatch(raw: str) -> None:
    if args.sim:
        return
    if _serial_port:
        try:    _serial_port.write(raw.encode())
        except Exception: pass
    else:
        _ble_queue.put(raw)


# ── BLE transport (HC-08 → Arduino) ──────────────────────────────────────────
async def _ble_main() -> None:
    global _ble_connected, _ble_status
    while _running:
        _ble_connected = False
        _ble_status    = 'Connecting to HC-08...'
        try:
            async with bleak.BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                _ble_connected = True
                _ble_status    = f'Connected  {HC08_ADDRESS}'
                while True:           # flush stale queue
                    try:    _ble_queue.get_nowait()
                    except queue.Empty: break
                await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                await asyncio.sleep(0.05)
                await client.write_gatt_char(
                    UART_CHAR_UUID, _SPEED_CMDS[_speed_level].encode(), response=True)
                while client.is_connected and _running:
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
        if _running:
            await asyncio.sleep(2.0)


def _ble_thread() -> None:
    asyncio.run(_ble_main())


# ── Filter coefficients — computed once at import time ────────────────────────
_SR  = 256
_NYQ = _SR / 2.0

# EOG 0.5–10 Hz: captures slow blink potentials on frontal channels (AF7/AF8)
_EOG_B, _EOG_A = butter(4, [0.5 / _NYQ, 10.0 / _NYQ], btype='band')

# EMG 20–100 Hz: muscle artifact for jaw clench on temporal channels (TP9/TP10)
_EMG_B, _EMG_A = butter(4, [20.0 / _NYQ, 100.0 / _NYQ], btype='band')


# ── Signal processor (alberthodo-style) ───────────────────────────────────────
class SignalProcessor:
    """
    Detects double blinks (EOG) and jaw clenches (EMG) from a rolling
    1-second EEG window supplied at 20 Hz.

    Muse 2 EEG channel layout (muselsl order):
        0 = TP9   left temporal   → EMG (jaw clench)
        1 = AF7   left frontal    → EOG (blink)
        2 = AF8   right frontal   → EOG (blink)
        3 = TP10  right temporal  → EMG (jaw clench)

    Algorithm sourced from alberthodo/bci-experimentations--muse-2:
      · blink_detector.py  — EOG negative-peak detection with timing constraints
      · signal_processor.py — bandpass preprocessing, adaptive baseline
    """

    BLINK_THRESH    = 150.0   # µV   minimum EOG peak amplitude for a voluntary blink
    BLINK_MIN_SEP   = 0.20    # s    shortest gap between two blinks of a double-blink
    BLINK_MAX_SEP   = 1.20    # s    longest gap still counted as double-blink
    BLINK_DEBOUNCE  = 0.15    # s    ignore re-detection within this window
    JAW_RATIO       = 3.0     # ×    EMG RMS must exceed this multiple of baseline
    JAW_COOLDOWN    = 1.5     # s    minimum time between jaw triggers
    DBLINK_COOLDOWN = 2.0     # s    minimum time between double-blink triggers

    def __init__(self) -> None:
        # Adaptive jaw baseline: rolling buffer of per-tick EMG RMS values
        # ~5 s at 20 Hz = 100 samples; primed with a low resting value
        self._jaw_baseline: deque = deque([5.0] * 100, maxlen=100)

        # Per-blink timestamps used for double-blink pairing
        self._blink_ts: deque = deque(maxlen=20)

        self._last_jaw_t:    float = -9999.0
        self._last_dblink_t: float = -9999.0

    def process(self, eeg: np.ndarray, t: float) -> None:
        """
        eeg : shape (4, N), N >= 64 samples, µV scale
        t   : current wall-clock time (seconds)

        Writes module-level _jaw_pending / _dblink_pending flags.
        """
        global _jaw_pending, _dblink_pending
        global _blink_count, _dblink_count, _jaw_count

        if eeg.shape[1] < 64:
            return

        # ── Jaw clench — EMG on TP9 (ch 0) and TP10 (ch 3) ─────────────────
        emg_flt = lfilter(_EMG_B, _EMG_A, eeg[[0, 3], :], axis=1)
        rms     = float(np.sqrt(np.mean(emg_flt ** 2)))

        # Compare against adaptive median baseline BEFORE updating it,
        # so a genuine clench doesn't corrupt the baseline immediately
        baseline = float(np.median(self._jaw_baseline))
        self._jaw_baseline.append(rms)

        if rms > self.JAW_RATIO * max(baseline, 1.0):
            if t - self._last_jaw_t > self.JAW_COOLDOWN:
                self._last_jaw_t = t
                _jaw_pending     = True
                _jaw_count      += 1

        # ── Blink / double-blink — EOG on AF7 (ch 1) and AF8 (ch 2) ────────
        eog_flt = lfilter(_EOG_B, _EOG_A, eeg[[1, 2], :], axis=1)

        # Average both frontal channels — voluntary blinks are bilateral
        eog_avg  = np.mean(eog_flt, axis=0)

        # Look at the trailing 128 ms (32 samples) for a fresh negative peak.
        # alberthodo uses negative deflections as the blink signature.
        trail    = eog_avg[-32:]
        peak_val = float(np.min(trail))

        if abs(peak_val) > self.BLINK_THRESH:
            if not self._blink_ts or (t - self._blink_ts[-1] > self.BLINK_DEBOUNCE):
                self._blink_ts.append(t)
                _blink_count += 1

                # Pair with the previous blink to detect a double-blink
                if (len(self._blink_ts) >= 2 and
                        t - self._last_dblink_t > self.DBLINK_COOLDOWN):
                    gap = t - self._blink_ts[-2]
                    if self.BLINK_MIN_SEP <= gap <= self.BLINK_MAX_SEP:
                        self._last_dblink_t = t
                        _dblink_pending     = True
                        _dblink_count      += 1


# ── Muse callbacks ────────────────────────────────────────────────────────────
_processor = SignalProcessor()
_eeg_buf   = np.zeros((4, _SR))   # rolling 1-second EEG window


def eeg_callback(data, timestamps) -> None:
    """Called by muselsl with a batch of EEG samples (shape: (5, n) incl. aux)."""
    global _eeg_buf
    # data rows: 0=TP9 1=AF7 2=AF8 3=TP10 4=AUX — take first 4
    new = np.array(data[:4], dtype=float)   # (4, n)
    n   = new.shape[1]
    if n >= _SR:
        _eeg_buf = new[:, -_SR:]
    else:
        _eeg_buf = np.roll(_eeg_buf, -n, axis=1)
        _eeg_buf[:, -n:] = new

    _processor.process(_eeg_buf, time.time())


def acc_callback(data, timestamps) -> None:
    """Called by muselsl with accelerometer samples (rows: ax ay az)."""
    global _pitch, _roll, _raw_pitch, _raw_roll, _imu_ts, _calibrated
    ax = float(data[0][-1])
    ay = float(data[1][-1])
    az = float(data[2][-1])
    _raw_pitch = math.degrees(math.atan2(ax, math.sqrt(ay ** 2 + az ** 2)))
    _raw_roll  = math.degrees(math.atan2(ay, az))
    _pitch     = _raw_pitch - _pitch_off
    _roll      = _raw_roll  - _roll_off
    _imu_ts    = time.time()
    if not _calibrated:
        calibrate()


# ── Calibration ───────────────────────────────────────────────────────────────
def calibrate() -> None:
    global _pitch_off, _roll_off, _calibrated
    _pitch_off  = _raw_pitch
    _roll_off   = _raw_roll
    _calibrated = True


# ── Control logic ─────────────────────────────────────────────────────────────
def _cycle_speed() -> None:
    global _speed_level
    _speed_level = (_speed_level + 1) % 3
    send_raw(_SPEED_CMDS[_speed_level])


def update_control(has_imu: bool) -> None:
    global _jaw_pending, _dblink_pending

    # Priority 1: jaw clench → immediate stop
    if _jaw_pending:
        _jaw_pending = False
        send_cmd('S')
        return

    # Priority 2: double blink → cycle speed, then fall through to tilt
    if _dblink_pending:
        _dblink_pending = False
        _cycle_speed()

    # Priority 3: head tilt steering
    if not has_imu:
        send_cmd('S')
        return

    fwd   = _pitch >  PITCH_FWD_THRESHOLD
    back  = _pitch < -PITCH_BACK_THRESHOLD
    left  = _roll  < -ROLL_THRESHOLD
    right = _roll  >  ROLL_THRESHOLD

    if   fwd  and left:   send_cmd('Q')
    elif fwd  and right:  send_cmd('E')
    elif back and left:   send_cmd('G')
    elif back and right:  send_cmd('H')
    elif fwd:             send_cmd('F')
    elif back:            send_cmd('B')
    elif left:            send_cmd('L')
    elif right:           send_cmd('R')
    else:                 send_cmd('S')


# ── Display ───────────────────────────────────────────────────────────────────
_CMD_LABEL = {
    'F':  '\033[92mFORWARD        ▲\033[0m',
    'B':  '\033[93mBACKWARD       ▼\033[0m',
    'L':  '\033[94mSPIN LEFT      ◄\033[0m',
    'R':  '\033[94mSPIN RIGHT     ►\033[0m',
    'Q':  '\033[96mFWD-LEFT       ◤\033[0m',
    'E':  '\033[96mFWD-RIGHT      ◥\033[0m',
    'G':  '\033[96mBCK-LEFT       ◣\033[0m',
    'H':  '\033[96mBCK-RIGHT      ◢\033[0m',
    'S':  '\033[91mSTOP           ■\033[0m',
    None: '\033[90m---\033[0m',
}

_HSI_COL = {1: '\033[92m', 2: '\033[93m', 4: '\033[91m', 0: '\033[90m'}
_HSI_LBL = {1: 'Good', 2: 'Med ', 4: 'Poor', 0: '--- '}


def _hsi_str(v: int) -> str:
    return f"{_HSI_COL.get(v, _HSI_COL[0])}{_HSI_LBL.get(v, '--- ')}\033[0m"


def _print_display(muse_ok: bool) -> None:
    pitch_lbl = 'NOD  ↓' if _pitch > 5 else ('LEAN ↑' if _pitch < -5 else 'Level •')
    roll_lbl  = 'TILT ◄' if _roll  < -5 else ('TILT ►' if _roll  >  5 else 'Level •')
    cal_str   = '\033[92mcalibrated\033[0m' if _calibrated else '\033[93mnot calibrated\033[0m'
    muse_col  = '\033[92m' if muse_ok else '\033[91m'
    muse_str  = 'Connected (muselsl)' if muse_ok else 'Disconnected'

    if args.sim:
        out_col, out_str = '\033[95m', 'SIM MODE — no hardware'
    elif args.com:
        out_col, out_str = '\033[92m', f'Serial  {args.com}'
    else:
        out_col = '\033[92m' if _ble_connected else '\033[91m'
        out_str = _ble_status

    spd_str  = f'[{_SPEED_DOTS[_speed_level]}]  {_SPEED_LABELS[_speed_level]}  (double-blink to cycle)'
    stat_str = (f'Blinks:{_blink_count:<4} Dbl:{_dblink_count:<4}(→spd)'
                f'  Jaw:{_jaw_count:<4}(→STOP)')
    sig_str  = (f'TP9:{_hsi_str(_hsi[0])}  AF7:{_hsi_str(_hsi[1])}'
                f'  AF8:{_hsi_str(_hsi[2])}  TP10:{_hsi_str(_hsi[3])}')

    sys.stdout.write('\033[2J\033[H')
    print(
        f"  ┌─ HYBRID BCI CONTROLLER  (muselsl + alberthodo detection) ─────┐\n"
        f"  │  Muse   {muse_col}{muse_str:<55}\033[0m│\n"
        f"  │  Output {out_col}{out_str:<55}\033[0m│\n"
        f"  │  Signal {sig_str:<55}│\n"
        f"  │                                                                │\n"
        f"  │  Pitch  {_pitch:+6.1f}°  {pitch_lbl}  (fwd/back ±{PITCH_FWD_THRESHOLD:.0f}°){'':>14}│\n"
        f"  │  Roll   {_roll:+6.1f}°  {roll_lbl}  (tilt L/R ±{ROLL_THRESHOLD:.0f}°){'':>12}│\n"
        f"  │  Speed  {_SPEED_COLS[_speed_level]}{spd_str:<55}\033[0m│\n"
        f"  │                                                                │\n"
        f"  │  CMD    {_CMD_LABEL.get(_last_cmd, _CMD_LABEL[None]):<54}│\n"
        f"  │                                                                │\n"
        f"  │  {stat_str:<63}│\n"
        f"  └────────────────────────────────────────────────────────────────┘\n"
        f"  Ctrl+C = quit  │  r = recalibrate ({cal_str})"
    )
    sys.stdout.flush()


# ── Keyboard listener ─────────────────────────────────────────────────────────
def _key_listener() -> None:
    import msvcrt
    while _running:
        if msvcrt.kbhit() and msvcrt.getch() == b'r':
            calibrate()
        time.sleep(0.05)


# ── Signal handler ────────────────────────────────────────────────────────────
def _handle_stop(sig, frame) -> None:
    global _running
    _running = False


signal.signal(signal.SIGINT,  _handle_stop)
signal.signal(signal.SIGTERM, _handle_stop)


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    global _running, _serial_port

    sys.stdout.write('\033[2J\033[H')
    print('  Hybrid BCI Controller  (alberthodo detection + gyro steering)')
    print('  ──────────────────────────────────────────────────────────────')
    if args.sim:
        print('  [SIM] No hardware — commands display-only')
    print()

    # Serial output setup
    if args.com and not args.sim:
        if not _HAS_SERIAL:
            print('  [ERROR] pyserial not installed.')
            return
        try:
            _serial_port = _pyserial.Serial(args.com, 9600, timeout=0.1)
            time.sleep(2.5)
            print(f'  Arduino ready on {args.com}')
        except Exception as e:
            print(f'  [ERROR] Cannot open {args.com}: {e}')
            return

    # Find Muse
    if args.mac:
        address = args.mac
        print(f'  Using MAC: {address}')
    else:
        print('  Scanning for Muse 2...')
        found = find_muse(backend='bleak')
        if not found:
            print('  No Muse found. Power it on and try again.')
            return
        address = found['address']
        print(f'  Found: {found.get("name", "Muse")}  ({address})')

    print('  Connecting...')
    muse = Muse(
        address=address,
        callback_eeg=eeg_callback,
        callback_acc=acc_callback,
        backend='bleak',
    )

    if not muse.connect():
        print('  Connection failed.')
        return

    muse.start()
    time.sleep(2.0)
    try:    muse.ask_control()
    except Exception: pass

    # Start output / keyboard threads
    if not args.sim and not args.com:
        threading.Thread(target=_ble_thread, daemon=True).start()
    threading.Thread(target=_key_listener, daemon=True).start()

    loop_dt   = 1.0 / LOOP_HZ
    disp_dt   = 1.0 / DISPLAY_HZ
    t_display = 0.0
    t_alive   = time.time()
    t_ctrl    = time.time()
    errors    = 0

    sys.stdout.write('\033[2J\033[H')

    while _running:
        now     = time.time()
        has_imu = (_imu_ts > 0) and (now - _imu_ts <= 2.0)

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
                    except Exception: pass
                    time.sleep(2.0)
                    if muse.connect():
                        muse.start(); time.sleep(1.0); errors = 0
                    else:
                        print('\n  Muse reconnect failed — exiting.')
                        break
            t_alive = now

        if now - t_ctrl > 2:
            try:    muse.ask_control()
            except Exception: pass
            t_ctrl = now

        update_control(has_imu)

        if now - t_display >= disp_dt:
            _print_display(muse_ok=has_imu)
            t_display = now

        elapsed = time.time() - now
        spare   = loop_dt - elapsed
        if spare > 0:
            time.sleep(spare)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    send_cmd('S')
    time.sleep(0.2)
    print('\n  Shutting down...')
    try:    muse.stop(); muse.disconnect()
    except Exception: pass
    if _serial_port:
        try:    _serial_port.close()
        except Exception: pass
    print('  Done.')


if __name__ == '__main__':
    main()
