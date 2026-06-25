#!/usr/bin/env python3
"""
muse_athena_car_controller.py
Muse S Athena EEG → Arduino car controller via HC-08 BLE.

Control scheme (head-tilt steering + blink/jaw drive):
  Single blink (both eyes)          → FORWARD  (latched)
  Double blink (×2)                 → STOP     (latched)
  Jaw clench                        → BACKWARD (latched)
  Head tilt L/R (roll > ROLL_THRESHOLD) → curved turn while moving (Q/E fwd · G/H bck)
  A clench is held JAW_CONFIRM_DELAY s; a blink in that window (blink bleeds into
  TP9/TP10) cancels it. A clench with no blink → BACKWARD.
  First 30 seconds                  → syncing: EEG + jaw baselines build, IMU roll zeroes

Jaw detection is the drift-proof per-channel detector from muse_athena_jaw_test.py:
high-frequency EMG on TP9 and TP10, each compared to its own adaptive baseline with
an MAD-based z-score trigger (JAW_K spreads above baseline). Both channels must jump
together, so blinks/eye movement (low-freq, on AF7/AF8) don't trigger it.

Two BLE devices connect independently and the dashboard shows each one's
status (Muse headset + Arduino HC-08). Either may still be connecting while
the other is live. Both keep retrying automatically if they aren't found.

Run:   eeg_env\\Scripts\\python.exe muse_athena_car_controller.py
"""

import asyncio
import sys
import signal
import time
import math
import threading
import queue
try:
    import msvcrt   # Windows: non-blocking keypress reads for the 'r' reset key
except ImportError:
    msvcrt = None
from collections import deque
import numpy as np
import bleak
from bleak import BleakClient, BleakScanner

from config import HC08_ADDRESS, UART_CHAR_UUID, DEFAULT_SPEED, ROLL_THRESHOLD

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
ACC_SCALE  = 0.0000610352   # raw → g
GYRO_SCALE = -0.0074768     # raw → deg/s


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
SYNC_DURATION = 30.0   # seconds before blink/tilt commands activate (after Muse connects)

# ── Blink detection tuning (ADAPTIVE) ─────────────────────────────────────────
# Unlike the fixed-µV detector, a blink must exceed the channel's *live* resting
# level by K × its noise spread (MAD). This self-scales to whatever amplitude the
# signal sits at, so a high baseline (e.g. ~700 µV on good contact) no longer
# makes every noise wobble read as a blink. Raise K = stricter (fewer blinks).
BLINK_K          = 6.0    # right eye (AF8): peak must clear baseline by K × noise
BLINK_K_LEFT     = 5.0    # left eye (AF7) is weaker → slightly easier (lower K)
BLINK_HIST_LEN   = 256    # samples (~4 s) of resting amplitude history per eye
BLINK_SPREAD_MIN = 0.05   # spread floor as a fraction of baseline (relative sens.)
FALL_FRAC   = 0.40   # spike ends when it falls this far back toward baseline
COOLDOWN    = 0.20   # s  refractory gap so a double-blink still registers twice
SHOW_MS     = 500
MIN_PEAK    = 216    # kept only for the EEG bar colouring (not used for detection)
# A railed electrode (no skin contact) sits pinned near the 1450 µV ceiling and its
# saturation noise reads as endless false blinks. At/above this we treat the channel
# as "no contact" and suppress blink detection entirely.
SATURATION = 1430.0

# ── Blink → command timing ────────────────────────────────────────────────────
BLINK_MERGE  = 0.18   # s  L+R of one physical blink merge into a single event
MULTI_WINDOW = 1.00   # s  window to tell a single blink from a double: 1× → FORWARD
                      #    (fires MULTI_WINDOW after the blink), 2× → STOP (fires at once
                      #    on the 2nd blink). Raise if a double-blink reads as two singles.

# ── Jaw clench detection (drift-proof per-channel TP9/TP10 EMG, from jaw test) ──
JAW_WIN        = 64     # samples (~0.25 s) of TP9/TP10 history
JAW_K          = 3.375  # jump must exceed baseline by K × noise spread (raise = stricter); lowered 25% from 4.5 for easier clench detection
JAW_SPREAD_MIN = 0.08   # spread floor as a fraction of baseline (keeps detection relative)
JAW_HIST_LEN   = 240    # samples of resting history per channel (~ a few seconds)
JAW_COOLDOWN   = 0.8    # s  minimum gap between counted jaw triggers
JAW_CONFIRM_DELAY = 0.25  # s  hold a detected clench this long; if a blink lands in the
                          #    window it was a blink bleeding into TP9/TP10 → FORWARD, not back

# ── Movement-start cooldowns ──────────────────────────────────────────────────
# When a FORWARD or BACKWARD movement begins, briefly reject new commands so
# blink/jaw noise can't bounce the drive state right after a move starts.
# Head-tilt steering is never affected (it only refines an already-active move).
FLIP_COOLDOWN = 0.20   # s  block the opposite direction (fwd↔back) this long
STOP_COOLDOWN = 0.06   # s  block STOP this long (kept short so a halt stays snappy)

# ── Drive state ───────────────────────────────────────────────────────────────
# 0 = STOP, 1 = FORWARD, 2 = BACKWARD
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


# ── Blink detector (adaptive, MAD-based) ──────────────────────────────────────
class BlinkDetector:
    """Adaptive per-eye blink detector. Instead of a fixed µV threshold, it tracks
    the channel's live resting amplitude (median) and noise spread (MAD) and fires
    only when a transient rises K × spread above that baseline, then falls back.
    Because the trigger self-scales, a high resting level (e.g. ~700 µV) no longer
    turns ordinary noise into a flood of false blinks. Mirrors the jaw detector."""

    def __init__(self, k: float = BLINK_K):
        self.k            = k               # left eye uses a lower K (easier)
        self.hist         = deque([10.0] * BLINK_HIST_LEN, maxlen=BLINK_HIST_LEN)
        self.baseline     = 10.0            # median resting amplitude (µV)
        self.spread       = 0.0             # MAD-based noise spread (µV)
        self.trig         = 0.0             # adaptive trigger level (µV)
        self.in_spike     = False
        self.peak         = 0.0
        self.count        = 0
        self.last_blink   = 0.0
        self.lit_until    = 0.0
        self.peak_display = 0.0
        self.saturated    = False

    def process(self, samples: np.ndarray) -> bool:
        val = float(np.max(np.abs(samples)))
        self.peak_display = val
        self.saturated = val >= SATURATION

        # Recompute the adaptive baseline + trigger from the resting history.
        self.baseline = float(np.median(self.hist))
        self.spread   = max(_mad(np.array(self.hist)), BLINK_SPREAD_MIN * self.baseline)
        self.trig     = self.baseline + self.k * self.spread

        if self.saturated:
            # Railed electrode = bad contact. Don't count saturation noise as
            # blinks; keep adapting so detection resumes on good contact.
            self.in_spike = False
            self.hist.append(val)
            return False

        over = val > self.trig
        # Learn resting history only from calm (non-spike) samples so a blink
        # can't inflate the baseline it is measured against.
        if not over and not self.in_spike:
            self.hist.append(val)

        if not self.in_spike:
            if over:
                self.in_spike = True
                self.peak = val
        else:
            if val > self.peak:
                self.peak = val
            fall_target = self.baseline + (self.peak - self.baseline) * FALL_FRAC
            if val < fall_target:
                self.in_spike = False
                now = time.time()
                if (now - self.last_blink) > COOLDOWN:
                    self.count     += 1
                    self.last_blink = now
                    self.lit_until  = now + SHOW_MS / 1000.0
                    return True
        return False

    def is_lit(self) -> bool:
        return time.time() < self.lit_until


# ── Jaw clench detector (drift-proof per-channel TP9/TP10 EMG) ─────────────────
def _mad(arr: np.ndarray) -> float:
    """Median absolute deviation, scaled to approximate one standard deviation."""
    med = np.median(arr)
    return float(np.median(np.abs(arr - med)) * 1.4826)


def _jaw_trigger(base: float, spread: float) -> float:
    """Adaptive trigger: JAW_K spreads above baseline, with a relative spread floor."""
    spread = max(spread, JAW_SPREAD_MIN * base)
    return base + JAW_K * spread


class JawDetector:
    """A clench shows up as a high-frequency EMG jump on BOTH TP9 and TP10 at once.
    Each channel is compared to its own adaptive baseline (median) + MAD-based
    trigger, so it tracks resting drift and rejects blinks (low-freq, on AF7/AF8)."""

    def __init__(self):
        self.tp9_buf    = deque(maxlen=JAW_WIN)
        self.tp10_buf   = deque(maxlen=JAW_WIN)
        self.emg9_hist  = deque([10.0] * JAW_HIST_LEN, maxlen=JAW_HIST_LEN)
        self.emg10_hist = deque([10.0] * JAW_HIST_LEN, maxlen=JAW_HIST_LEN)
        self.emg9       = 0.0
        self.emg10      = 0.0
        self.emg9_base  = 10.0
        self.emg10_base = 10.0
        self.emg9_trig  = 0.0
        self.emg10_trig = 0.0
        self.last_jaw   = 0.0
        self.count      = 0
        self.lit_until  = 0.0

    def process(self, arr: np.ndarray, calibrating: bool) -> bool:
        """arr = decoded EEG (4 samples, 4 channels). TP9 = col 0, TP10 = col 3.
        Returns True on a counted clench. While calibrating, only learns baselines."""
        self.tp9_buf.extend(arr[:, 0].tolist())
        self.tp10_buf.extend(arr[:, 3].tolist())
        if len(self.tp9_buf) < 16:
            return False

        a9  = np.array(self.tp9_buf)
        a10 = np.array(self.tp10_buf)
        self.emg9  = float(np.mean(np.abs(np.diff(a9))))
        self.emg10 = float(np.mean(np.abs(np.diff(a10))))

        self.emg9_base  = float(np.median(self.emg9_hist))
        self.emg10_base = float(np.median(self.emg10_hist))
        self.emg9_trig  = _jaw_trigger(self.emg9_base,  _mad(np.array(self.emg9_hist)))
        self.emg10_trig = _jaw_trigger(self.emg10_base, _mad(np.array(self.emg10_hist)))

        emg9_over  = self.emg9  > self.emg9_trig
        emg10_over = self.emg10 > self.emg10_trig

        # Learn each baseline: always while calibrating, then only from non-clench
        # samples so a clench can't inflate it.
        if calibrating or not emg9_over:
            self.emg9_hist.append(self.emg9)
        if calibrating or not emg10_over:
            self.emg10_hist.append(self.emg10)

        now = time.time()
        # Only TP9 needs to cross its trigger to count as a clench; TP10 is still
        # tracked for the meter/baseline but no longer required.
        if (not calibrating
                and emg9_over
                and now - self.last_jaw > JAW_COOLDOWN):
            self.last_jaw  = now
            self.count    += 1
            self.lit_until = now + 0.8
            return True
        return False

    def is_lit(self) -> bool:
        return time.time() < self.lit_until


# ── Shared state ──────────────────────────────────────────────────────────────
_lock      = threading.Lock()
left_det   = BlinkDetector(BLINK_K_LEFT)   # AF7 — weaker eye, lower K (easier)
right_det  = BlinkDetector()               # AF8 — default K
jaw_det    = JawDetector()
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]
_pkt_count = 0
running    = True

# Drive state — 0=STOP  1=FORWARD  2=BACKWARD
_drive_state = 0
# Movement-start cooldowns: while now < these, the matching command is dropped.
_flip_cooldown_until = 0.0   # blocks fwd↔back flips for FLIP_COOLDOWN
_stop_cooldown_until = 0.0   # blocks STOP for STOP_COOLDOWN

# Head-tilt (IMU roll) steering state ─────────────────────────────────────────
_imu_roll_raw     = 0.0    # raw roll from the accelerometer (deg)
_imu_roll         = 0.0    # roll after subtracting the resting offset
_imu_roll_samples = []     # roll readings gathered during sync → resting offset
_roll_offset      = 0.0
_imu_calibrated   = False
_imu_ts           = 0.0    # time of last IMU packet (used for staleness check)

# Blink-burst resolution: 2 blinks → FORWARD, 3+ blinks → STOP (1 = ignored).
_blink_last_event  = 0.0   # last merged blink (de-dupes L+R of one physical blink)
_blink_burst_count = 0     # blinks counted in the current burst
_blink_burst_last  = 0.0   # time of the most recent blink in the burst
_blink_events      = 0     # debug: total blink events that passed the merge de-dupe
_last_blink_action = '—'   # debug: last resolved burst result (FWD / STOP / ignore1)

# Jaw clench resolution. A clench is held JAW_CONFIRM_DELAY before committing to
# BACKWARD; if a blink fires inside that window it was a blink bleeding into
# TP9/TP10 (→ FORWARD), so the pending clench is cancelled.
_jaw_pending      = False
_jaw_pending_time = 0.0
_last_blink_fire  = 0.0   # timestamp of the last both-eye blink

# Connection status (shown on dashboard) ──────────────────────────────────────
_muse_connected = False
_muse_status    = 'Waiting...'
_hc08_connected = False
_hc08_status    = 'Waiting...'

# _sync_start = inf means sync hasn't started yet → _is_syncing() stays True until
# it's set to time.time() once the Muse has connected and started streaming.
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
                # Flush any stale commands
                while True:
                    try:    _hc08_queue.get_nowait()
                    except queue.Empty: break
                # Send initial stop, then lock in the same speed as the keyboard tester
                await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                await client.write_gatt_char(
                    UART_CHAR_UUID, DEFAULT_SPEED.encode(), response=True)
                while client.is_connected and running:
                    try:
                        raw = _hc08_queue.get_nowait()
                        await client.write_gatt_char(
                            UART_CHAR_UUID, raw.encode(), response=True)
                    except queue.Empty:
                        pass
                    await asyncio.sleep(0.02)
                # On shutdown (Ctrl-C), halt the car before disconnecting. The send
                # loop above has already exited, so a queued 'S' would never go out —
                # write STOP directly here while the link is still up.
                if not running and client.is_connected:
                    try:
                        await client.write_gatt_char(
                            UART_CHAR_UUID, b'S', response=True)
                    except Exception:
                        pass
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


# ── Blink-burst resolver ──────────────────────────────────────────────────────
def _register_blink(now: float) -> None:
    """Count blinks into a burst. L+R of one physical blink (within BLINK_MERGE)
    collapse to one. A blink also cancels a just-detected clench: a blink bleeds
    into TP9/TP10, so a clench within JAW_CONFIRM_DELAY of a blink was that blink."""
    global _blink_last_event, _blink_burst_count, _blink_burst_last
    global _last_blink_fire, _jaw_pending, _blink_events
    if now - _blink_last_event < BLINK_MERGE:
        return   # same physical blink firing twice (e.g. AF7 then AF8)
    _blink_last_event = now
    _last_blink_fire  = now
    _blink_events    += 1                 # debug counter
    if _blink_burst_count > 0 and (now - _blink_burst_last) <= MULTI_WINDOW:
        _blink_burst_count += 1          # another blink in the same burst
    else:
        _blink_burst_count = 1           # new burst
    _blink_burst_last = now


def _resolve_blinks():
    """Run in control loop. 1 blink → FORWARD, 2+ → STOP. STOP fires the moment a
    2nd blink lands (instant, no wait). A lone blink fires FORWARD only once the
    MULTI_WINDOW closes with no 2nd blink. Returns (forward, stop)."""
    global _blink_burst_count, _last_blink_action
    now     = time.time()
    forward = stop = False
    with _lock:
        if _blink_burst_count >= 2:
            stop = True
            _last_blink_action = f'STOP({_blink_burst_count})'
            _blink_burst_count = 0
        elif _blink_burst_count == 1 and (now - _blink_burst_last) > MULTI_WINDOW:
            forward = True
            _last_blink_action = 'FWD(1)'
            _blink_burst_count = 0
    return forward, stop


# ── Sensor callback ───────────────────────────────────────────────────────────
def on_sensor(handle, data: bytearray):
    global _pkt_count, _jaw_pending, _jaw_pending_time
    global _imu_roll_raw, _imu_roll, _imu_roll_samples
    global _roll_offset, _imu_calibrated, _imu_ts

    _pkt_count += 1
    subpackets = parse_payload(bytes(data))
    for tag, stype, raw in subpackets:

        if tag == 0x11:   # EEG 4-channel
            arr     = decode_eeg_4ch(raw)
            blink_l = left_det.process(arr[:, 1])    # AF7 — left eye
            blink_r = right_det.process(arr[:, 2])   # AF8 — right eye
            now     = time.time()
            jaw_fired = jaw_det.process(arr, _is_syncing())   # TP9/TP10 clench

            with _lock:
                for ch in range(4):
                    _eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))
            if (blink_l or blink_r) and not _is_syncing():
                with _lock:
                    _register_blink(now)
            # A clench arms a pending BACKWARD; it commits only after
            # JAW_CONFIRM_DELAY with no blink (a coincident blink → it was bleed → FWD).
            if jaw_fired and not _is_syncing():
                with _lock:
                    _jaw_pending      = True
                    _jaw_pending_time = now

        elif tag == 0x47:   # ACCGYRO → head-tilt (roll) steering
            imu = decode_accgyro(raw)
            ay, az = float(imu[-1, 1]), float(imu[-1, 2])
            _imu_roll_raw = math.degrees(math.atan2(ay, az))
            _imu_ts = time.time()
            if _is_syncing():
                _imu_roll_samples.append(_imu_roll_raw)   # gather resting offset
            else:
                if not _imu_calibrated and _imu_roll_samples:
                    _roll_offset    = float(np.mean(_imu_roll_samples))
                    _imu_calibrated = True
                _imu_roll = _imu_roll_raw - _roll_offset


def on_ctrl(handle, data: bytearray):
    pass


# ── Control logic (called at 20 Hz from UI thread) ────────────────────────────
def update_control() -> None:
    global _drive_state, _jaw_pending
    global _flip_cooldown_until, _stop_cooldown_until

    forward, stop = _resolve_blinks()   # 1 blink → FORWARD · 2+ blinks → STOP

    now = time.time()
    # A detected clench commits to BACKWARD immediately. The JawDetector already
    # gates on BOTH TP9+TP10 over their adaptive trigger plus JAW_COOLDOWN, so a
    # clench is trustworthy on its own — no blink-bleed confirm delay needed.
    jaw_back = False
    with _lock:
        if _jaw_pending:
            _jaw_pending = False
            jaw_back = True

    # Movement-start cooldowns: while a fresh move is settling, drop the commands
    # that would fight it. FLIP_COOLDOWN blocks the opposite direction (so noise
    # can't flip fwd↔back); STOP_COOLDOWN briefly blocks STOP. Head-tilt steering
    # below is untouched. STOP after its short window still wins for safety.
    if now < _flip_cooldown_until:
        forward  = False
        jaw_back = False
    if now < _stop_cooldown_until:
        stop = False

    # Apply latched drive commands. Order matters: STOP (double-blink) wins ties.
    # Starting FORWARD/BACKWARD arms both cooldowns from this instant.
    if jaw_back:
        _drive_state = 2          # BACKWARD
        _flip_cooldown_until = now + FLIP_COOLDOWN
        _stop_cooldown_until = now + STOP_COOLDOWN
    if forward:
        _drive_state = 1          # FORWARD
        _flip_cooldown_until = now + FLIP_COOLDOWN
        _stop_cooldown_until = now + STOP_COOLDOWN
    if stop:
        _drive_state = 0          # STOP

    # Safety: never drive unless the Muse is connected and the sync is done
    if not _muse_connected or _is_syncing():
        send_cmd('S')
        return

    now = time.time()

    # Head-tilt steering: tilt ear-to-shoulder past ROLL_THRESHOLD curves L/R while moving.
    has_imu = _imu_ts > 0 and (now - _imu_ts) <= 2.0
    left  = has_imu and _imu_roll < -ROLL_THRESHOLD
    right = has_imu and _imu_roll >  ROLL_THRESHOLD

    if _drive_state == 1:        # FORWARD
        if   left:  send_cmd('Q')   # fwd-left
        elif right: send_cmd('E')   # fwd-right
        else:       send_cmd('F')
    elif _drive_state == 2:      # BACKWARD
        if   left:  send_cmd('G')   # bck-left
        elif right: send_cmd('H')   # bck-right
        else:       send_cmd('B')
    else:                        # STOP
        send_cmd('S')


# ── Terminal display ──────────────────────────────────────────────────────────
DISPLAY_LINES = 20
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


def _blink_tag(det: BlinkDetector) -> str:
    if det.saturated:
        return '\033[1;97;41m NO CONTACT \033[0m'   # railed electrode — fix fit
    return '\033[1;97;44m BLINK \033[0m' if det.is_lit() else '\033[90m ------ \033[0m'


def _conn_cell(connected: bool, status: str) -> str:
    if connected:
        return f'\033[92m● CONNECTED\033[0m  {status}'
    return f'\033[91m○ waiting\033[0m    {status}'


def _frac_bar(frac: float, width: int = 12) -> str:
    """Bar that fills toward a trigger (frac >= 1.0 = over)."""
    frac   = max(0.0, min(frac, 1.0))
    filled = int(frac * width)
    col    = '\033[91m' if frac >= 1.0 else ('\033[93m' if frac > 0.6 else '\033[92m')
    return col + '█' * filled + '\033[90m' + '░' * (width - filled) + '\033[0m'


def _roll_str() -> str:
    """Head-tilt (roll) steering indicator for the dashboard."""
    has_imu = _imu_ts > 0 and (time.time() - _imu_ts) <= 2.0
    if not has_imu:
        return '\033[90mIMU: no data yet\033[0m'
    roll = _imu_roll
    if roll < -ROLL_THRESHOLD:
        dir_lbl = '\033[1;96m◄ TURN LEFT\033[0m'
    elif roll > ROLL_THRESHOLD:
        dir_lbl = '\033[1;96mTURN RIGHT ►\033[0m'
    else:
        dir_lbl = '\033[92mSTRAIGHT\033[0m'
    return f'Roll {roll:+6.1f}°  {dir_lbl}'


def _jaw_str() -> str:
    """Jaw-clench meter: per-channel fraction toward the adaptive trigger."""
    j   = jaw_det
    f9  = (j.emg9  - j.emg9_base)  / (j.emg9_trig  - j.emg9_base)  if j.emg9_trig  > j.emg9_base  else 0.0
    f10 = (j.emg10 - j.emg10_base) / (j.emg10_trig - j.emg10_base) if j.emg10_trig > j.emg10_base else 0.0
    tag = '\033[1;97;41m JAW! \033[0m' if j.is_lit() else '\033[90m ---- \033[0m'
    return f'TP9 {_frac_bar(f9)}  TP10 {_frac_bar(f10)}  {tag} cnt:{j.count}'


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

    # Banner: depends on connection + sync state
    if not _muse_connected:
        banner = '  \033[91m── WAITING FOR MUSE HEADSET ───────────────────────────────\033[0m'
    elif syncing:
        banner = (f'  \033[93m── SYNCING  {rem:.0f}s remaining  '
                  f'(keep headset still, let EEG settle) ──\033[0m')
    else:
        banner = '  \033[92m── ACTIVE ─────────────────────────────────────────────────\033[0m'

    # Drive state line
    if not _muse_connected or syncing:
        drive_str = (f'  Drive:  {DRIVE_COLORS[ds]}{DRIVE_LABELS[ds]}\033[0m'
                     f'   \033[90m(not active yet)\033[0m')
    else:
        drive_str = (f'  Drive:  {DRIVE_COLORS[ds]}{DRIVE_LABELS[ds]}\033[0m'
                     f'   \033[90mblink → FWD · jaw → BACK · 2× blink → STOP\033[0m')

    rows = [
        f'  \033[1mMuse S Athena\033[0m — Car Controller   [{time.strftime("%H:%M:%S")}]',
        f'  Muse   {_conn_cell(_muse_connected, _muse_status)}   \033[90mpkts:{_pkt_count}\033[0m',
        f'  HC-08  {_conn_cell(_hc08_connected, _hc08_status)}',
        f'',
        banner,
        f'',
        drive_str,
        f'  Steer:  {_roll_str()}',
        f'',
        f'  \033[96m── EEG ─────────────────────────────────────────────────────\033[0m',
        f'  TP9   {eeg[0]:7.1f} µV   {_eeg_bar(eeg[0])}',
        f'  AF7   {lv:7.1f} µV   {_eeg_bar(lv)}  {_blink_tag(left_det)} L:{left_det.count}  base:{lb:.0f} trig:{left_det.trig:.0f}',
        f'  AF8   {rv:7.1f} µV   {_eeg_bar(rv)}  {_blink_tag(right_det)} R:{right_det.count}  base:{rb:.0f} trig:{right_det.trig:.0f}',
        f'  TP10  {eeg[3]:7.1f} µV   {_eeg_bar(eeg[3])}',
        f'',
        f'  \033[96m── JAW ─────────────────────────────────────────────────────\033[0m',
        f'  {_jaw_str()}',
        f'',
        f'  CMD: {_CMD_DESC.get(_last_cmd, "---")}   '
        f'\033[90mblinks: burst={_blink_burst_count} events={_blink_events} last={_last_blink_action}\033[0m',
        f'  \033[90mCtrl-C quit · r=recalibrate · blink=FWD · 2 blinks=STOP · jaw=BACK · head-tilt=turn\033[0m',
    ]
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()


def _reset_calibration() -> None:
    """'r' key handler: restart the 30 s calibration live so the user can re-tune
    on the fly. Re-zeros head-tilt roll, rebuilds blink + jaw baselines, clears
    blink counts/bursts, and parks the car in STOP for safety."""
    global left_det, right_det, jaw_det, _sync_start
    global _imu_calibrated, _imu_roll_samples, _roll_offset, _imu_roll
    global _blink_burst_count, _blink_events, _last_blink_action, _drive_state
    global _flip_cooldown_until, _stop_cooldown_until
    with _lock:
        left_det  = BlinkDetector(BLINK_K_LEFT)
        right_det = BlinkDetector()
        jaw_det   = JawDetector()
        _imu_calibrated    = False
        _imu_roll_samples  = []
        _roll_offset       = 0.0
        _imu_roll          = 0.0
        _blink_burst_count = 0
        _blink_events      = 0
        _last_blink_action = '—'
        _drive_state       = 0
        _flip_cooldown_until = 0.0
        _stop_cooldown_until = 0.0
        _sync_start        = time.time()   # restart the 30 s sync
    send_cmd('S')


def _ui_thread() -> None:
    """Dashboard + control loop. Runs the whole program lifetime so connection
    status is visible even while the Muse/HC-08 are still connecting."""
    sys.stdout.write('\n' * DISPLAY_LINES)
    ctrl_tick = time.time()
    while running:
        now = time.time()
        # 'r' restarts calibration live so the user can re-tune on the fly.
        if msvcrt is not None and msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ('r', 'R'):
                _reset_calibration()
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
def _mlog(msg: str):
    """Append a timestamped line to muse_debug.log (TUI hides stdout)."""
    try:
        with open('muse_debug.log', 'a', encoding='utf-8') as f:
            f.write(f'{time.strftime("%H:%M:%S")} {msg}\n')
    except Exception:
        pass


async def _notify_with_retry(client, uuid, callback, label, attempts=5):
    """start_notify, retrying on transient WinRT 'Unreachable' errors.

    On Windows the descriptor write that enables notifications often fails the
    first time(s) with BleakError: 'Unreachable' even though the device is
    connected. Retrying after a short pause usually succeeds — this mirrors how
    OpenMuse tolerates the same failure."""
    for i in range(1, attempts + 1):
        try:
            await client.start_notify(uuid, callback)
            _mlog(f'notify {label} OK (attempt {i})')
            return
        except Exception as e:
            _mlog(f'notify {label} attempt {i} failed: {type(e).__name__}: {e}')
            if i == attempts:
                raise
            await asyncio.sleep(0.4)


async def muse_main():
    global running, _sync_start, _muse_connected, _muse_status

    while running:
        _muse_connected = False
        try:
            _muse_status = 'Scanning for any Muse headset...'
            _mlog('scanning...')
            # Scan for ALL nearby BLE devices and pick the first Muse, regardless
            # of MAC address — so any Muse headset works, not just one specific unit.
            found = await BleakScanner.discover(timeout=12.0)
            device = next(
                (d for d in found if d.name and 'muse' in d.name.lower()),
                None,
            )
            if device is None:
                _muse_status = 'No Muse found — is the headset on? Retrying...'
                _mlog(f'no muse; saw {[(d.address, d.name) for d in found]}')
                await asyncio.sleep(2.0)
                continue

            _muse_status = f'Found {device.name} ({device.address}) — connecting...'
            _mlog(f'found {device.name} {device.address}; connecting...')
            async with BleakClient(device, timeout=20.0) as client:
                _mlog('connected (entered context)')
                # Windows can hand back an empty/stale GATT table on the first
                # connect to a newly-paired headset, which makes start_notify
                # raise BleakCharacteristicNotFoundError. Verify the chars are
                # actually discovered first; if not, drop and retry (the next
                # attempt connects with a warm cache and succeeds).
                svcs = client.services
                n_chars = sum(len(s.characteristics) for s in svcs)
                _mlog(f'services discovered: {n_chars} characteristics')
                if (svcs.get_characteristic(CTRL_UUID) is None or
                        svcs.get_characteristic(SENSOR_UUID) is None):
                    _muse_status = 'GATT not ready — reconnecting...'
                    _mlog('GATT not ready (chars missing) — reconnecting')
                    await asyncio.sleep(1.0)
                    continue

                _muse_status = 'Connected — running init sequence...'
                _mlog('starting notify on CTRL_UUID')
                await _notify_with_retry(client, CTRL_UUID, on_ctrl, 'CTRL')

                for step, cmd_bytes, delay in INIT_SEQ:
                    _mlog(f'init step {step}')
                    await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
                    await asyncio.sleep(delay)
                    if step == SUBSCRIBE_AFTER_STEP:
                        _mlog('subscribing to SENSOR_UUID')
                        await _notify_with_retry(client, SENSOR_UUID, on_sensor, 'SENSOR')

                # Start the 30s sync only on the first successful connect
                if _sync_start == float('inf'):
                    _sync_start = time.time()

                _muse_connected = True
                _muse_status    = f'Streaming  {device.name} ({device.address})'
                _mlog('init complete — streaming')

                # Keep the connection alive until it drops or we quit
                while client.is_connected and running:
                    await asyncio.sleep(0.2)
                _mlog(f'connection loop exited (is_connected={client.is_connected})')

        except Exception as e:
            _mlog(f'EXCEPTION {type(e).__name__}: {e}')
            _muse_status = f'Lost / failed ({type(e).__name__}) — retrying...'

        _muse_connected = False
        if running:
            await asyncio.sleep(2.0)

    # On quit, make sure the car is told to stop
    send_cmd('S')


def main():
    threading.Thread(target=_hc08_thread, daemon=True).start()
    threading.Thread(target=_ui_thread,   daemon=True).start()
    try:
        asyncio.run(muse_main())
    except KeyboardInterrupt:
        pass
    # Let the HC-08 thread push its final STOP over BLE before we exit.
    time.sleep(0.6)
    print("\nStopped.")


if __name__ == "__main__":
    main()
