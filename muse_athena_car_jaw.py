#!/usr/bin/env python3
"""
muse_athena_car_jaw.py
Copy of muse_athena_car_controller.py WITH jaw-clench detection added.
(The original controller is left untouched — this is a separate test variant.)

Control scheme:
  Single blink (either eye)  → toggle drive direction: FORWARD ↔ BACKWARD
  Double blink                → STOP
  Jaw clench                  → EMERGENCY STOP (top priority)
  Head tilt left  (>15°)      → curved turn LEFT   (fwd-left Q / bck-left G)
  Head tilt right (>15°)      → curved turn RIGHT  (fwd-right E / bck-right H)
  First 30 seconds            → syncing period: EEG baseline builds, IMU calibrates

Jaw detection is scipy-free: it measures high-frequency EMG energy on the
temporal channels (TP9/TP10) as the mean absolute sample-to-sample change,
compared against an adaptive baseline. The dashboard shows a live JAW meter
so you can watch the value vs. the trigger threshold and tune it.

Tune these if it triggers too easily / never triggers:
  JAW_RATIO      — how many × the baseline counts as a clench (raise = stricter)
  JAW_ABS_FLOOR  — absolute floor so quiet signal can't fire (raise = stricter)

Run:   python muse_athena_car_jaw.py
"""

import asyncio
import sys
import signal
import time
import threading
import math
import queue
from collections import deque
import numpy as np
import bleak
from bleak import BleakClient, BleakScanner

from config import HC08_ADDRESS, UART_CHAR_UUID, ROLL_THRESHOLD

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

# ── Athena packet constants (from amused-py) ───────────────────────────────────
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

EEG_SCALE  = 1450.0 / 16383.0
ACC_SCALE  = 0.0000610352
GYRO_SCALE = -0.0074768


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

# ── Control constants ─────────────────────────────────────────────────────────
SYNC_DURATION = 30.0   # seconds before blink/tilt/jaw commands activate

# ── Blink detection tuning ────────────────────────────────────────────────────
RISE_THRESH = 120
MIN_PEAK    = 180
FALL_FRAC   = 0.40
COOLDOWN    = 0.20
SHOW_MS     = 500

# ── Blink → command timing ────────────────────────────────────────────────────
BLINK_MERGE   = 0.18
DOUBLE_WINDOW = 0.45

# ── Jaw clench detection (scipy-free high-frequency EMG metric) ────────────────
JAW_WIN       = 64     # samples (~0.25 s) of TP9/TP10 history
JAW_RATIO     = 4.0    # metric must exceed this × adaptive baseline
JAW_ABS_FLOOR = 60.0   # absolute floor so quiet signal can't trigger
JAW_COOLDOWN  = 1.2    # s  minimum gap between jaw triggers

# ── Drive state ───────────────────────────────────────────────────────────────
# 0 = STOP, 1 = FORWARD, 2 = BACKWARD
DRIVE_CMDS   = ['S', 'F', 'B']
DRIVE_LABELS = ['STOP  ■', 'FORWARD ▲', 'BACKWARD ▼']
DRIVE_COLORS = ['\033[91m', '\033[92m', '\033[93m']


# ── Signal decoders ───────────────────────────────────────────────────────────
def _unpack_bits_lsb(data: bytes) -> list:
    bits = []
    for byte in data:
        for bit in range(8):
            bits.append((byte >> bit) & 1)
    return bits


def decode_eeg_4ch(data: bytes) -> np.ndarray:
    """14-bit LSB-first packed → shape (4 samples, 4 channels) in µV"""
    bits = _unpack_bits_lsb(data[:28])
    raw = []
    for i in range(16):
        v = 0
        for b in range(14):
            if bits[i * 14 + b]:
                v |= (1 << b)
        raw.append(v)
    return np.array(raw, dtype=np.float32).reshape(4, 4) * EEG_SCALE


def decode_accgyro(data: bytes) -> np.ndarray:
    """16-bit signed LE → shape (3 samples, 6 channels): accX/Y/Z in g, gyroX/Y/Z in deg/s"""
    raw = np.frombuffer(data[:36], dtype="<i2").reshape(3, 6).astype(np.float32)
    result = raw.copy()
    result[:, 0:3] *= ACC_SCALE
    result[:, 3:6] *= GYRO_SCALE
    return result


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
        print(f'No Muse found. Seen: {[(d.address, d.name) for d in devices]}')
    return muse


# ── Blink detector ────────────────────────────────────────────────────────────
class BlinkDetector:
    def __init__(self):
        self.baseline     = None
        self.in_spike     = False
        self.peak         = 0.0
        self.count        = 0
        self.last_blink   = 0.0
        self.lit_until    = 0.0
        self.peak_display = 0.0

    def process(self, samples: np.ndarray) -> bool:
        val = float(np.max(np.abs(samples)))
        self.peak_display = val
        if self.baseline is None:
            self.baseline = val
            return False
        if not self.in_spike:
            self.baseline = 0.96 * self.baseline + 0.04 * val
            if (val - self.baseline) > RISE_THRESH and val > MIN_PEAK:
                self.in_spike = True
                self.peak = val
        else:
            if val > self.peak:
                self.peak = val
            fall_target = self.baseline + (self.peak - self.baseline) * FALL_FRAC
            if val < fall_target:
                self.in_spike = False
                self.baseline = 0.96 * self.baseline + 0.04 * val
                now = time.time()
                if self.peak > MIN_PEAK and (now - self.last_blink) > COOLDOWN:
                    self.count     += 1
                    self.last_blink = now
                    self.lit_until  = now + SHOW_MS / 1000.0
                    return True
        return False

    def is_lit(self) -> bool:
        return time.time() < self.lit_until


# ── Shared state ──────────────────────────────────────────────────────────────
_lock      = threading.Lock()
left_det   = BlinkDetector()
right_det  = BlinkDetector()
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]
_pkt_count = 0
running    = True

# IMU
_imu_roll_raw     = 0.0
_imu_roll         = 0.0
_imu_roll_samples = []
_roll_offset      = 0.0
_imu_calibrated   = False
_imu_ts           = 0.0

# Drive state — 0=STOP  1=FORWARD  2=BACKWARD
_drive_state = 0
_last_dir    = 1

# Blink event resolution
_blink_last_event   = 0.0
_blink_pending      = False
_blink_pending_time = 0.0
_blink_double_flag  = False

# Jaw clench → emergency STOP
_tp9_buf       = deque(maxlen=JAW_WIN)
_tp10_buf      = deque(maxlen=JAW_WIN)
_jaw_hist      = deque([10.0] * 120, maxlen=120)
_last_jaw      = 0.0
_jaw_count     = 0
_jaw_lit_until = 0.0
_jaw_metric    = 0.0    # live value (for meter)
_jaw_baseline  = 10.0   # live baseline (for meter)
_jaw_stop_pending = False

# Connection status
_muse_connected = False
_muse_status    = 'Waiting...'
_hc08_connected = False
_hc08_status    = 'Waiting...'

_sync_start: float = float('inf')


# ── HC-08 BLE output thread ───────────────────────────────────────────────────
_hc08_queue = queue.SimpleQueue()
_last_cmd: str | None = None


def send_cmd(cmd: str) -> None:
    global _last_cmd
    if cmd == _last_cmd:
        return
    _last_cmd = cmd
    _hc08_queue.put(cmd)


async def _hc08_ble_main() -> None:
    global _hc08_connected, _hc08_status
    while running:
        _hc08_connected = False
        _hc08_status    = 'Connecting...'
        try:
            async with bleak.BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                _hc08_connected = True
                _hc08_status    = f'Connected  {HC08_ADDRESS}'
                while True:
                    try:    _hc08_queue.get_nowait()
                    except queue.Empty: break
                await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                while client.is_connected and running:
                    try:
                        raw = _hc08_queue.get_nowait()
                        await client.write_gatt_char(
                            UART_CHAR_UUID, raw.encode(), response=True)
                    except queue.Empty:
                        pass
                    await asyncio.sleep(0.02)
        except Exception as e:
            _hc08_status = f'Not found — retrying ({type(e).__name__})'
        _hc08_connected = False
        if running:
            await asyncio.sleep(2.0)


def _hc08_thread() -> None:
    asyncio.run(_hc08_ble_main())


# ── Sync helpers ──────────────────────────────────────────────────────────────
def _is_syncing() -> bool:
    return (time.time() - _sync_start) < SYNC_DURATION


def _sync_remaining() -> float:
    return max(0.0, SYNC_DURATION - (time.time() - _sync_start))


# ── Blink event registration (called from Muse notification thread) ───────────
def _register_blink(now: float) -> None:
    global _blink_last_event, _blink_pending, _blink_pending_time, _blink_double_flag
    if now - _blink_last_event < BLINK_MERGE:
        return
    _blink_last_event = now
    if _blink_pending and (now - _blink_pending_time) <= DOUBLE_WINDOW:
        _blink_pending     = False
        _blink_double_flag = True
    else:
        _blink_pending      = True
        _blink_pending_time = now


def _resolve_blinks() -> None:
    global _blink_pending, _blink_double_flag, _drive_state, _last_dir
    now = time.time()
    with _lock:
        double = _blink_double_flag
        _blink_double_flag = False
        single = False
        if _blink_pending and (now - _blink_pending_time) > DOUBLE_WINDOW:
            _blink_pending = False
            single = True

    if double:
        _drive_state = 0
    elif single:
        if _drive_state == 0:
            _drive_state = _last_dir
        else:
            _drive_state = 2 if _drive_state == 1 else 1
            _last_dir    = _drive_state


# ── Jaw clench detection ──────────────────────────────────────────────────────
def _detect_jaw(eeg: np.ndarray) -> None:
    """High-frequency EMG burst on TP9/TP10 → emergency STOP."""
    global _last_jaw, _jaw_count, _jaw_lit_until
    global _jaw_metric, _jaw_baseline, _jaw_stop_pending
    _tp9_buf.extend(eeg[:, 0].tolist())
    _tp10_buf.extend(eeg[:, 3].tolist())
    if len(_tp9_buf) < 16:
        return
    a9  = np.array(_tp9_buf)
    a10 = np.array(_tp10_buf)
    metric = float(np.mean(np.abs(np.diff(a9))) + np.mean(np.abs(np.diff(a10))))
    base   = float(np.median(_jaw_hist))
    _jaw_metric   = metric
    _jaw_baseline = base

    # Only fold non-clench samples into the baseline so a clench can't inflate it
    if base < 1.0 or metric < JAW_RATIO * base:
        _jaw_hist.append(metric)

    now = time.time()
    if (not _is_syncing()
            and metric > JAW_RATIO * base
            and metric > JAW_ABS_FLOOR
            and now - _last_jaw > JAW_COOLDOWN):
        _last_jaw      = now
        _jaw_count    += 1
        _jaw_lit_until = now + 0.8
        with _lock:
            _jaw_stop_pending = True


# ── Sensor callback ───────────────────────────────────────────────────────────
def on_sensor(handle, data: bytearray):
    global _pkt_count
    global _imu_roll_raw, _imu_roll, _imu_roll_samples
    global _roll_offset, _imu_calibrated, _imu_ts

    _pkt_count += 1
    subpackets = parse_payload(bytes(data))
    for tag, stype, raw in subpackets:

        if tag == 0x11:   # EEG 4-channel
            arr     = decode_eeg_4ch(raw)
            blink_l = left_det.process(arr[:, 1])    # AF7 — left eye
            blink_r = right_det.process(arr[:, 2])   # AF8 — right eye
            with _lock:
                for ch in range(4):
                    _eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))
            _detect_jaw(arr)
            if (blink_l or blink_r) and not _is_syncing():
                with _lock:
                    _register_blink(time.time())

        elif tag == 0x47:   # ACCGYRO
            imu  = decode_accgyro(raw)
            ax, ay, az = float(imu[-1, 0]), float(imu[-1, 1]), float(imu[-1, 2])
            _imu_roll_raw = math.degrees(math.atan2(ay, az))
            _imu_ts = time.time()

            if _is_syncing():
                _imu_roll_samples.append(_imu_roll_raw)
            else:
                if not _imu_calibrated and _imu_roll_samples:
                    _roll_offset    = float(np.mean(_imu_roll_samples))
                    _imu_calibrated = True
                _imu_roll = _imu_roll_raw - _roll_offset


def on_ctrl(handle, data: bytearray):
    pass


# ── Control logic (called at 20 Hz from UI thread) ────────────────────────────
def update_control() -> None:
    global _jaw_stop_pending, _drive_state

    # Top priority: jaw clench → emergency stop
    with _lock:
        jaw_stop = _jaw_stop_pending
        _jaw_stop_pending = False
    if jaw_stop:
        _drive_state = 0

    _resolve_blinks()

    if not _muse_connected or _is_syncing():
        send_cmd('S')
        return

    has_imu = _imu_ts > 0 and (time.time() - _imu_ts) <= 2.0
    left    = has_imu and _imu_roll < -ROLL_THRESHOLD
    right   = has_imu and _imu_roll >  ROLL_THRESHOLD

    if _drive_state == 1:        # FORWARD
        if   left:  send_cmd('Q')
        elif right: send_cmd('E')
        else:       send_cmd('F')
    elif _drive_state == 2:      # BACKWARD
        if   left:  send_cmd('G')
        elif right: send_cmd('H')
        else:       send_cmd('B')
    else:                        # STOP
        send_cmd('S')


# ── Terminal display ──────────────────────────────────────────────────────────
DISPLAY_LINES = 21
_first_frame  = True

_CMD_DESC = {
    'F': 'FORWARD      ▲', 'B': 'BACKWARD     ▼',
    'Q': 'FWD-LEFT     ◤', 'E': 'FWD-RIGHT    ◥',
    'G': 'BCK-LEFT     ◣', 'H': 'BCK-RIGHT    ◢',
    'S': 'STOP         ■',
}


def _eeg_bar(val: float, width: int = 22) -> str:
    frac   = min(val / 500.0, 1.0)
    filled = int(frac * width)
    col    = '\033[91m' if val > MIN_PEAK else ('\033[93m' if val > MIN_PEAK * 0.5 else '\033[92m')
    return col + '█' * filled + '\033[90m' + '░' * (width - filled) + '\033[0m'


def _jaw_bar(metric: float, trigger: float, width: int = 22) -> str:
    frac   = min(metric / trigger, 1.0) if trigger > 0 else 0.0
    filled = int(frac * width)
    col    = '\033[91m' if frac >= 1.0 else ('\033[93m' if frac > 0.6 else '\033[92m')
    return col + '█' * filled + '\033[90m' + '░' * (width - filled) + '\033[0m'


def _blink_tag(det: BlinkDetector) -> str:
    return '\033[1;97;44m BLINK \033[0m' if det.is_lit() else '\033[90m ------ \033[0m'


def _conn_cell(connected: bool, status: str) -> str:
    if connected:
        return f'\033[92m● CONNECTED\033[0m  {status}'
    return f'\033[91m○ waiting\033[0m    {status}'


def draw():
    global _first_frame
    if not _first_frame:
        sys.stdout.write(f'\033[{DISPLAY_LINES}A\033[J')
    _first_frame = False

    with _lock:
        eeg = list(_eeg_vals)

    lv      = left_det.peak_display
    rv      = right_det.peak_display
    lb      = left_det.baseline  or 0.0
    rb      = right_det.baseline or 0.0
    syncing = _is_syncing()
    rem     = _sync_remaining()
    ds      = _drive_state
    roll    = _imu_roll
    has_imu = _imu_ts > 0 and (time.time() - _imu_ts) <= 2.0
    jaw_lit = time.time() < _jaw_lit_until
    jaw_trigger = max(JAW_RATIO * _jaw_baseline, JAW_ABS_FLOOR)

    if not _muse_connected:
        banner = '  \033[91m── WAITING FOR MUSE HEADSET ───────────────────────────────\033[0m'
    elif syncing:
        banner = (f'  \033[93m── SYNCING  {rem:.0f}s remaining  '
                  f'(keep headset still, let EEG settle) ──\033[0m')
    else:
        banner = '  \033[92m── ACTIVE ─────────────────────────────────────────────────\033[0m'

    if not _muse_connected or syncing:
        drive_str = (f'  Drive:  {DRIVE_COLORS[ds]}{DRIVE_LABELS[ds]}\033[0m'
                     f'   \033[90m(not active yet)\033[0m')
    else:
        other = 'BACKWARD' if ds == 1 else 'FORWARD'
        hint  = f'blink → {other}' if ds != 0 else 'blink → GO'
        drive_str = (f'  Drive:  {DRIVE_COLORS[ds]}{DRIVE_LABELS[ds]}\033[0m'
                     f'   \033[90m1×{hint} · 2×→STOP · jaw→STOP\033[0m')

    if has_imu:
        if roll < -ROLL_THRESHOLD:
            dir_lbl = '\033[96m◄ TURN LEFT \033[0m'
        elif roll > ROLL_THRESHOLD:
            dir_lbl = '\033[96mTURN RIGHT ►\033[0m'
        else:
            dir_lbl = '\033[90mlevel\033[0m      '
        imu_str = (f'  Roll  {roll:+6.1f}°  {dir_lbl}   '
                   f'CMD: {_CMD_DESC.get(_last_cmd, "---")}')
    else:
        imu_str = '  IMU: \033[90mno data yet\033[0m'

    jaw_flag = '\033[1;97;41m JAW! \033[0m' if jaw_lit else '\033[90m ---- \033[0m'
    jaw_str  = (f'  EMG {_jaw_metric:6.1f}  {_jaw_bar(_jaw_metric, jaw_trigger)}  '
                f'{jaw_flag}  trig>{jaw_trigger:.0f}  base:{_jaw_baseline:.0f}  Jaw:{_jaw_count}')

    rows = [
        f'  \033[1mMuse S Athena\033[0m — Car + Jaw Test   [{time.strftime("%H:%M:%S")}]',
        f'  Muse   {_conn_cell(_muse_connected, _muse_status)}   \033[90mpkts:{_pkt_count}\033[0m',
        f'  HC-08  {_conn_cell(_hc08_connected, _hc08_status)}',
        f'',
        banner,
        f'',
        drive_str,
        f'',
        f'  \033[96m── EEG ─────────────────────────────────────────────────────\033[0m',
        f'  TP9   {eeg[0]:7.1f} µV   {_eeg_bar(eeg[0])}',
        f'  AF7   {lv:7.1f} µV   {_eeg_bar(lv)}  {_blink_tag(left_det)} L:{left_det.count}  base:{lb:.0f}',
        f'  AF8   {rv:7.1f} µV   {_eeg_bar(rv)}  {_blink_tag(right_det)} R:{right_det.count}  base:{rb:.0f}',
        f'  TP10  {eeg[3]:7.1f} µV   {_eeg_bar(eeg[3])}',
        f'',
        f'  \033[96m── JAW CLENCH (TP9/TP10 high-freq EMG) ─────────────────────\033[0m',
        jaw_str,
        f'',
        f'  \033[96m── IMU ─────────────────────────────────────────────────────\033[0m',
        imu_str,
        f'',
        f'  \033[90mCtrl-C quit · 1 blink=toggle · 2 blinks=stop · jaw=STOP · tilt=turn\033[0m',
    ]
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()


def _ui_thread() -> None:
    sys.stdout.write('\n' * DISPLAY_LINES)
    ctrl_tick = time.time()
    while running:
        now = time.time()
        if now - ctrl_tick >= 0.05:
            update_control()
            ctrl_tick = now
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
    global running, _sync_start, _muse_connected, _muse_status

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

                if _sync_start == float('inf'):
                    _sync_start = time.time()

                _muse_connected = True
                _muse_status    = f'Streaming  {device.name} ({device.address})'

                while client.is_connected and running:
                    await asyncio.sleep(0.2)

        except Exception as e:
            _muse_status = f'Lost / failed ({type(e).__name__}) — retrying...'

        _muse_connected = False
        if running:
            await asyncio.sleep(2.0)

    send_cmd('S')


def main():
    threading.Thread(target=_hc08_thread, daemon=True).start()
    threading.Thread(target=_ui_thread,   daemon=True).start()
    try:
        asyncio.run(muse_main())
    except KeyboardInterrupt:
        pass
    time.sleep(0.3)
    print("\nStopped.")


if __name__ == "__main__":
    main()
