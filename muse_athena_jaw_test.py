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

Detection (scipy-free, drift-proof — NO fixed absolute thresholds):
A clench shows up as a big JUMP in the high-frequency EMG of BOTH temporal
channels at once. The EMG metric per channel is the mean absolute
sample-to-sample change (a high-frequency energy proxy). Only high-frequency
energy is used, so blinks and eye movement (low-frequency, mostly on AF7/AF8)
don't trigger it.

Each channel has its own adaptive baseline (median of recent resting samples)
and noise spread (MAD). A clench is flagged only when BOTH TP9 and TP10 jump
above  baseline + JAW_K × spread.
This is fully relative, so it tracks the resting level as it drifts upward over
a session (e.g. TP9 ~30 early → ~70-80 after a few minutes) instead of using a
fixed floor that goes stale.

Tune at the top:
  JAW_K          — noise-spreads above baseline a channel must jump (raise = stricter)
  JAW_SPREAD_MIN — spread floor as a fraction of baseline (keeps it relative)

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
# Detection is RELATIVE (z-score style), so it follows the resting level as it
# drifts: fire when a metric exceeds  baseline + JAW_K × spread.  No fixed floor.
# Both temporal channels (TP9 AND TP10) use the high-frequency EMG metric, which
# rejects blinks/eye movement (those are low-frequency, mostly on AF7/AF8).
CALIB_DURATION = 10.0   # s  warm-up: build baseline, don't count detections yet
JAW_WIN        = 64     # samples (~0.25 s) of TP9/TP10 history
JAW_K          = 3.0    # jump must exceed baseline by K × noise spread (raise = stricter)
JAW_SPREAD_MIN = 0.08   # spread floor as a fraction of baseline (keeps detection relative, not absolute)
JAW_HIST_LEN   = 240    # samples of resting history per metric (~ a few seconds)
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


async def _find_muse_device(timeout: float = 12.0):
    devices = await BleakScanner.discover(timeout=timeout)
    muse = next((d for d in devices if d.name and 'muse' in d.name.lower()), None)
    if muse is None:
        _mlog(f'no muse; saw {[(d.address, d.name) for d in devices]}')
    return muse


# ── Shared state ──────────────────────────────────────────────────────────────
_lock      = threading.Lock()
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]   # TP9, AF7, AF8, TP10 peak |amplitude|
_pkt_count = 0
running    = True

_tp9_buf  = deque(maxlen=JAW_WIN)
_tp10_buf = deque(maxlen=JAW_WIN)
_emg9_hist  = deque([10.0] * JAW_HIST_LEN, maxlen=JAW_HIST_LEN)   # resting TP9 EMG
_emg10_hist = deque([10.0] * JAW_HIST_LEN, maxlen=JAW_HIST_LEN)   # resting TP10 EMG

_emg9       = 0.0    # TP9 high-freq EMG (live)
_emg10      = 0.0    # TP10 high-freq EMG (live)
_emg9_base  = 10.0   # adaptive TP9 baseline (live)
_emg10_base = 10.0   # adaptive TP10 baseline (live)
_emg9_trig  = 0.0    # live TP9 trigger level
_emg10_trig = 0.0    # live TP10 trigger level
_last_jaw  = 0.0
_jaw_count = 0
_jaw_lit_until = 0.0
_events: deque = deque(maxlen=LOG_LINES)   # (time_str, emg9+emg10, delta)

_muse_connected = False
_muse_status    = 'Waiting...'
_start_time     = time.time()


def _mad(arr: np.ndarray) -> float:
    """Median absolute deviation, scaled to approximate one standard deviation."""
    med = np.median(arr)
    return float(np.median(np.abs(arr - med)) * 1.4826)


def _trigger(base: float, spread: float) -> float:
    """Adaptive trigger: JAW_K spreads above baseline, with a relative spread floor."""
    spread = max(spread, JAW_SPREAD_MIN * base)
    return base + JAW_K * spread


def _calibrating() -> bool:
    return (time.time() - _start_time) < CALIB_DURATION


def _calib_remaining() -> float:
    return max(0.0, CALIB_DURATION - (time.time() - _start_time))


# ── Sensor callback ───────────────────────────────────────────────────────────
def on_sensor(handle, data: bytearray):
    global _pkt_count, _emg9, _emg10
    global _emg9_base, _emg10_base, _emg9_trig, _emg10_trig
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

        # Per-channel high-frequency EMG (mean |sample-to-sample change|).
        # High-freq only → blinks / eye movement (low-freq) don't show up here.
        emg9   = float(np.mean(np.abs(np.diff(a9))))
        emg10  = float(np.mean(np.abs(np.diff(a10))))

        emg9_base  = float(np.median(_emg9_hist))
        emg10_base = float(np.median(_emg10_hist))
        emg9_trig  = _trigger(emg9_base,  _mad(np.array(_emg9_hist)))
        emg10_trig = _trigger(emg10_base, _mad(np.array(_emg10_hist)))

        emg9_over  = emg9  > emg9_trig
        emg10_over = emg10 > emg10_trig

        with _lock:
            _emg9, _emg10           = emg9, emg10
            _emg9_base, _emg10_base = emg9_base, emg10_base
            _emg9_trig, _emg10_trig = emg9_trig, emg10_trig

        # Learn each baseline: always while calibrating, then only from samples
        # that aren't over that channel's trigger, so a clench can't inflate it.
        if _calibrating() or not emg9_over:
            _emg9_hist.append(emg9)
        if _calibrating() or not emg10_over:
            _emg10_hist.append(emg10)

        # A clench must jump on BOTH temporal channels at once.
        now = time.time()
        if (not _calibrating()
                and emg9_over and emg10_over
                and now - _last_jaw > JAW_COOLDOWN):
            _last_jaw      = now
            _jaw_count    += 1
            _jaw_lit_until = now + 0.8
            total = emg9 + emg10
            delta = (emg9 - emg9_base) + (emg10 - emg10_base)
            with _lock:
                _events.appendleft((time.strftime("%H:%M:%S"), total, delta))


def on_ctrl(handle, data: bytearray):
    pass


# ── Display ───────────────────────────────────────────────────────────────────
DISPLAY_LINES = 25
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
        eeg        = list(_eeg_vals)
        emg9       = _emg9
        emg10      = _emg10
        emg9_base  = _emg9_base
        emg10_base = _emg10_base
        emg9_trig  = _emg9_trig
        emg10_trig = _emg10_trig
        events     = list(_events)

    emg9_frac  = (emg9  - emg9_base)  / (emg9_trig  - emg9_base)  if emg9_trig  > emg9_base  else 0.0
    emg10_frac = (emg10 - emg10_base) / (emg10_trig - emg10_base) if emg10_trig > emg10_base else 0.0
    jaw_lit    = time.time() < _jaw_lit_until
    emg9_over  = emg9  > emg9_trig
    emg10_over = emg10 > emg10_trig
    above      = emg9_over and emg10_over

    if not _muse_connected:
        banner = '  \033[91m── WAITING FOR MUSE HEADSET ───────────────────────────────\033[0m'
    elif _calibrating():
        banner = (f'  \033[93m── CALIBRATING  {_calib_remaining():.0f}s  '
                  f'(relax your jaw, stay still) ──\033[0m')
    else:
        banner = '  \033[92m── ARMED — clench to test ──────────────────────────────────\033[0m'

    jaw_flag = '\033[1;97;41m JAW! \033[0m' if jaw_lit else (
               '\033[1;30;103m over \033[0m' if above else '\033[90m ---- \033[0m')

    over_tag = '\033[92mOVER\033[0m'
    off_tag  = '\033[90m -- \033[0m'
    tp9_tag  = over_tag if emg9_over  else off_tag
    tp10_tag = over_tag if emg10_over else off_tag

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
        f'  \033[96m── JAW EMG (clench must jump on BOTH channels) ─────────────\033[0m',
        f'  TP9  {emg9:6.1f}  {_bar(emg9_frac, 24)}  {tp9_tag}',
        f'  TP10 {emg10:6.1f}  {_bar(emg10_frac, 24)}  {tp10_tag}  {jaw_flag}',
        f'  TP9>{emg9_trig:5.1f}(b{emg9_base:4.0f})   TP10>{emg10_trig:5.1f}(b{emg10_base:4.0f})   '
        f'count:{_jaw_count}',
        f'',
        f'  \033[96m── Detections (latest first) ───────────────────────────────\033[0m',
        log_rows[0], log_rows[1], log_rows[2], log_rows[3], log_rows[4], log_rows[5],
        f'',
        f'  \033[90mCtrl-C quit · clench to trigger · raise/lower JAW_K to tune (now {JAW_K:.1f})\033[0m',
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
                _muse_status = 'Scanning for any Muse headset...'
            device = await _find_muse_device(timeout=12.0)
            if device is None:
                _muse_status = 'Not found — is the headset on? Retrying...'
                await asyncio.sleep(2.0)
                continue

            _muse_status = f'Found {device.name} ({device.address}) — connecting...'
            async with BleakClient(device.address, timeout=20.0) as client:
                _muse_status = 'Connected — running init sequence...'
                await client.start_notify(CTRL_UUID, on_ctrl)
                for step, cmd_bytes, delay in INIT_SEQ:
                    await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
                    await asyncio.sleep(delay)
                    if step == SUBSCRIBE_AFTER_STEP:
                        await client.start_notify(SENSOR_UUID, on_sensor)

                _start_time     = time.time()   # (re)start calibration on connect
                _muse_connected = True
                _muse_status    = f'Streaming  {device.name} ({device.address})'

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
