#!/usr/bin/env python3
"""
muse_athena_jaw_test.py
Standalone JAW-CLENCH detection tester for the Muse S Athena.

NO Arduino, NO car control — it only connects to the headset and shows every
signal the jaw detector uses, so you can judge how accurate/reliable it is.

What it shows:
  · Raw amplitude of all 4 EEG channels (TP9, AF7, AF8, TP10)
  · Per-channel EMG energy on the temporal channels (TP9 / TP10)
  · Combined jaw metric, adaptive baseline, live ratio, trigger threshold
  · A meter that fills toward the threshold, and a JAW! flash on a detection
  · A running log of the last detections with timestamp + peak value

Detection (scipy-free): jaw muscles inject high-frequency noise into TP9/TP10.
The metric is the mean absolute sample-to-sample change (a high-frequency
energy proxy). A clench is flagged when it jumps far above the resting baseline.

Tune at the top:
  JAW_RATIO      — × baseline that counts as a clench   (raise = stricter)
  JAW_ABS_FLOOR  — absolute floor so quiet signal can't fire (raise = stricter)

Run:   python muse_athena_jaw_test.py        (python3 on the Pi)
"""

import asyncio
import sys
import signal
import time
import threading
from collections import deque
import numpy as np
from bleak import BleakClient, BleakScanner

# Enable ANSI VT100 escapes on Windows so the live dashboard renders
if sys.platform == 'win32':
    try:
        import ctypes
        _k32 = ctypes.windll.kernel32
        _k32.SetConsoleMode(_k32.GetStdHandle(-11), 7)
    except Exception:
        pass

# ── Muse S Athena BLE ─────────────────────────────────────────────────────────
MUSE_ADDR   = "00:55:DA:B9:FC:10"
CTRL_UUID   = "273e0001-4c4d-454d-96be-f03bac821358"
SENSOR_UUID = "273e0013-4c4d-454d-96be-f03bac821358"

HEADER_SIZE = 14
SENSOR_CONFIG = {
    0x11: ("EEG",     4,  4,  28),
    0x12: ("EEG",     8,  2,  28),
    0x34: ("OPTICS",  4,  3,  30),
    0x35: ("OPTICS",  8,  2,  40),
    0x36: ("OPTICS",  16, 1,  40),
    0x47: ("ACCGYRO", 6,  3,  36),
    0x88: ("BATTERY", 1,  1, 188),
    0x98: ("BATTERY", 1,  1,  20),
}
EEG_SCALE = 1450.0 / 16383.0


def encode_cmd(cmd: str) -> bytes:
    encoded = cmd.encode("utf-8") + b"\n"
    return bytes([len(encoded) + 1]) + encoded


INIT_SEQ = [
    ("v6",     encode_cmd("v6"),    0.05),
    ("s",      encode_cmd("s"),     0.05),
    ("h",      encode_cmd("h"),     0.10),
    ("p21",    encode_cmd("p21"),   0.05),
    ("s2",     encode_cmd("s"),     0.10),   # subscribe SENSOR_UUID after this step
    ("dc001a", encode_cmd("dc001"), 0.05),
    ("L1a",    encode_cmd("L1"),    0.05),
    ("h2",     encode_cmd("h"),     0.10),
    ("p1034",  encode_cmd("p1034"), 0.05),
    ("s3",     encode_cmd("s"),     0.10),
    ("dc001b", encode_cmd("dc001"), 0.05),
    ("L1b",    encode_cmd("L1"),    0.10),
]
SUBSCRIBE_AFTER_STEP = "s2"

# ── Jaw detection tuning ──────────────────────────────────────────────────────
# The metric rests around ~170 and a clench adds ~30, so detection is ADDITIVE:
# fire when the metric rises JAW_MARGIN above the adaptive baseline.
CALIB_DURATION = 10.0   # s  warm-up: build baseline, don't count detections yet
JAW_WIN        = 64     # samples (~0.25 s) of TP9/TP10 history
JAW_MARGIN     = 20.0   # metric must exceed baseline by this much to count a clench
JAW_ABS_FLOOR  = 150.0  # absolute guard floor (well below the ~170 resting level)
JAW_COOLDOWN   = 0.8    # s  minimum gap between counted detections
LOG_LINES      = 6      # how many recent detections to keep on screen


# ── Decoders ──────────────────────────────────────────────────────────────────
def _unpack_bits_lsb(data: bytes) -> list:
    bits = []
    for byte in data:
        for bit in range(8):
            bits.append((byte >> bit) & 1)
    return bits


def decode_eeg_4ch(data: bytes) -> np.ndarray:
    bits = _unpack_bits_lsb(data[:28])
    raw = []
    for i in range(16):
        v = 0
        for b in range(14):
            if bits[i * 14 + b]:
                v |= (1 << b)
        raw.append(v)
    return np.array(raw, dtype=np.float32).reshape(4, 4) * EEG_SCALE


def parse_payload(payload: bytes) -> list:
    results = []
    if len(payload) < HEADER_SIZE + 1:
        return results
    tag = payload[9]
    cfg = SENSOR_CONFIG.get(tag)
    if cfg is None:
        return results
    data_len = cfg[3]
    data_end = HEADER_SIZE + data_len
    if data_end > len(payload):
        return results
    results.append((tag, cfg[0], payload[HEADER_SIZE:data_end]))
    offset = data_end
    while offset + 5 < len(payload):
        tag = payload[offset]
        cfg = SENSOR_CONFIG.get(tag)
        if cfg is None:
            break
        data_len   = cfg[3]
        data_start = offset + 5
        data_end   = data_start + data_len
        if data_end > len(payload):
            break
        results.append((tag, cfg[0], payload[data_start:data_end]))
        offset = data_end
    return results


# ── Shared state ──────────────────────────────────────────────────────────────
_lock      = threading.Lock()
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]   # TP9, AF7, AF8, TP10 peak |amplitude|
_pkt_count = 0
running    = True

_tp9_buf  = deque(maxlen=JAW_WIN)
_tp10_buf = deque(maxlen=JAW_WIN)
_jaw_hist = deque([10.0] * 120, maxlen=120)

_metric    = 0.0     # combined jaw metric (live)
_emg9      = 0.0     # TP9 EMG energy (live)
_emg10     = 0.0     # TP10 EMG energy (live)
_baseline  = 10.0    # adaptive baseline (live)
_last_jaw  = 0.0
_jaw_count = 0
_jaw_lit_until = 0.0
_events: deque = deque(maxlen=LOG_LINES)   # (time_str, metric, ratio)

_muse_connected = False
_muse_status    = 'Waiting...'
_start_time     = time.time()


def _calibrating() -> bool:
    return (time.time() - _start_time) < CALIB_DURATION


def _calib_remaining() -> float:
    return max(0.0, CALIB_DURATION - (time.time() - _start_time))


# ── Sensor callback ───────────────────────────────────────────────────────────
def on_sensor(handle, data: bytearray):
    global _pkt_count, _metric, _emg9, _emg10, _baseline
    global _last_jaw, _jaw_count, _jaw_lit_until

    _pkt_count += 1
    for tag, stype, raw in parse_payload(bytes(data)):
        if tag != 0x11:          # only EEG needed for jaw
            continue
        arr = decode_eeg_4ch(raw)   # (4 samples, 4 channels)

        _tp9_buf.extend(arr[:, 0].tolist())
        _tp10_buf.extend(arr[:, 3].tolist())

        with _lock:
            for ch in range(4):
                _eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))

        if len(_tp9_buf) < 16:
            continue

        a9  = np.array(_tp9_buf)
        a10 = np.array(_tp10_buf)
        emg9   = float(np.mean(np.abs(np.diff(a9))))
        emg10  = float(np.mean(np.abs(np.diff(a10))))
        metric = emg9 + emg10
        base   = float(np.median(_jaw_hist))

        with _lock:
            _emg9, _emg10, _metric, _baseline = emg9, emg10, metric, base

        trigger = base + JAW_MARGIN

        # Learn the baseline: always while calibrating (so it reaches the true
        # resting level fast), then exclude clenches so they can't inflate it.
        if _calibrating() or base < 1.0 or metric < trigger:
            _jaw_hist.append(metric)

        now = time.time()
        if (not _calibrating()
                and metric > trigger
                and metric > JAW_ABS_FLOOR
                and now - _last_jaw > JAW_COOLDOWN):
            _last_jaw      = now
            _jaw_count    += 1
            _jaw_lit_until = now + 0.8
            delta = metric - base
            with _lock:
                _events.appendleft((time.strftime("%H:%M:%S"), metric, delta))


def on_ctrl(handle, data: bytearray):
    pass


# ── Display ───────────────────────────────────────────────────────────────────
DISPLAY_LINES = 23
_first_frame  = True


def _bar(frac: float, width: int = 24) -> str:
    frac   = max(0.0, min(frac, 1.0))
    filled = int(frac * width)
    col    = '\033[91m' if frac >= 1.0 else ('\033[93m' if frac > 0.6 else '\033[92m')
    return col + '█' * filled + '\033[90m' + '░' * (width - filled) + '\033[0m'


def draw():
    global _first_frame
    if not _first_frame:
        sys.stdout.write(f'\033[{DISPLAY_LINES}A\033[J')
    _first_frame = False

    with _lock:
        eeg    = list(_eeg_vals)
        metric = _metric
        emg9   = _emg9
        emg10  = _emg10
        base   = _baseline
        events = list(_events)

    trigger = base + JAW_MARGIN
    delta   = metric - base
    jaw_lit = time.time() < _jaw_lit_until
    above   = metric > trigger

    if not _muse_connected:
        banner = '  \033[91m── WAITING FOR MUSE HEADSET ───────────────────────────────\033[0m'
    elif _calibrating():
        banner = (f'  \033[93m── CALIBRATING  {_calib_remaining():.0f}s  '
                  f'(relax your jaw, stay still) ──\033[0m')
    else:
        banner = '  \033[92m── ARMED — clench to test ──────────────────────────────────\033[0m'

    jaw_flag = '\033[1;97;41m JAW! \033[0m' if jaw_lit else (
               '\033[1;30;103m over \033[0m' if above else '\033[90m ---- \033[0m')

    # EEG amplitude bars (scale ~500 µV full)
    ch_names = ['TP9 ', 'AF7 ', 'AF8 ', 'TP10']
    eeg_rows = []
    for i, nm in enumerate(ch_names):
        tag = '\033[96m(jaw)\033[0m' if i in (0, 3) else '     '
        eeg_rows.append(f'  {nm} {eeg[i]:7.1f} µV  {_bar(eeg[i] / 500.0, 18)} {tag}')

    # Detection log (fixed LOG_LINES rows)
    log_rows = []
    for i in range(LOG_LINES):
        if i < len(events):
            ts, mv, dv = events[i]
            log_rows.append(f'    \033[91m●\033[0m {ts}   metric {mv:6.1f}   Δ {dv:+6.1f}')
        else:
            log_rows.append('    \033[90m·\033[0m')

    muse_cell = '\033[92m● CONNECTED\033[0m' if _muse_connected else '\033[91m○ waiting\033[0m'

    rows = [
        f'  \033[1mMuse S Athena\033[0m — Jaw Clench Detection Test   [{time.strftime("%H:%M:%S")}]',
        f'  Muse: {muse_cell}  {_muse_status}   \033[90mpkts:{_pkt_count}\033[0m',
        f'',
        banner,
        f'',
        f'  \033[96m── EEG channels ────────────────────────────────────────────\033[0m',
        eeg_rows[0],
        eeg_rows[1],
        eeg_rows[2],
        eeg_rows[3],
        f'',
        f'  \033[96m── JAW METRIC (TP9/TP10 high-freq EMG) ─────────────────────\033[0m',
        f'  EMG {metric:6.1f}  {_bar(delta / JAW_MARGIN, 24)}  {jaw_flag}',
        f'  TP9:{emg9:6.1f}  TP10:{emg10:6.1f}   base:{base:5.1f}   '
        f'Δ:{delta:+6.1f}   trig>{trigger:.0f}   count:{_jaw_count}',
        f'',
        f'  \033[96m── Detections (latest first) ───────────────────────────────\033[0m',
        log_rows[0], log_rows[1], log_rows[2], log_rows[3], log_rows[4], log_rows[5],
        f'',
        f'  \033[90mCtrl-C quit · clench jaw to trigger · raise/lower JAW_MARGIN to tune (now {JAW_MARGIN:.0f})\033[0m',
    ]
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()


def _ui_thread() -> None:
    sys.stdout.write('\n' * DISPLAY_LINES)
    while running:
        draw()
        time.sleep(0.05)


# ── Signal handler ────────────────────────────────────────────────────────────
def _sig_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT,  _sig_handler)
signal.signal(signal.SIGTERM, _sig_handler)


# ── Muse connection (retries until found) ─────────────────────────────────────
async def muse_main():
    global running, _muse_connected, _muse_status, _start_time

    while running:
        _muse_connected = False
        try:
            _muse_status = f'Scanning for {MUSE_ADDR}...'
            device = await BleakScanner.find_device_by_address(MUSE_ADDR, timeout=12.0)
            if device is None:
                _muse_status = 'Not found — is the headset on? Retrying...'
                await asyncio.sleep(2.0)
                continue

            _muse_status = 'Found — connecting...'
            async with BleakClient(device, timeout=20.0) as client:
                _muse_status = 'Connected — running init sequence...'
                await client.start_notify(CTRL_UUID, on_ctrl)
                for step, cmd_bytes, delay in INIT_SEQ:
                    await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
                    await asyncio.sleep(delay)
                    if step == SUBSCRIBE_AFTER_STEP:
                        await client.start_notify(SENSOR_UUID, on_sensor)

                _start_time     = time.time()   # (re)start calibration on connect
                _muse_connected = True
                _muse_status    = f'Streaming  {MUSE_ADDR}'

                while client.is_connected and running:
                    await asyncio.sleep(0.2)

        except Exception as e:
            _muse_status = f'Lost / failed ({type(e).__name__}) — retrying...'

        _muse_connected = False
        if running:
            await asyncio.sleep(2.0)


def main():
    threading.Thread(target=_ui_thread, daemon=True).start()
    try:
        asyncio.run(muse_main())
    except KeyboardInterrupt:
        pass
    time.sleep(0.2)
    print("\nStopped.")


if __name__ == "__main__":
    main()
