#!/usr/bin/env python3
"""
muse_athena_car_controller.py
Muse S Athena EEG → Arduino car controller via HC-08 BLE.

Control scheme (blink + jaw + wink steering):
  Both eyes (normal blink)          → single: FORWARD · double: STOP  (latched)
  Jaw clench                        → BACKWARD  (latched)
  Left eye ONLY (AF7 wink)          → curved turn LEFT  (Q fwd / G bck), brief pulse
  Right eye ONLY (AF8 wink)         → curved turn RIGHT (E fwd / H bck), brief pulse
                                       (winks only steer while moving; pulse = WINK_PULSE_DUR)
  A clench is held JAW_CONFIRM_DELAY s; a blink in that window (blink bleeds into
  TP9/TP10) cancels it → FORWARD. A clench with no blink → BACKWARD.
  First 30 seconds                  → syncing period: EEG + jaw baselines build

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
import threading
import queue
from collections import deque
import numpy as np
import bleak
from bleak import BleakClient, BleakScanner

from config import HC08_ADDRESS, UART_CHAR_UUID, DEFAULT_SPEED, WINK_COINCIDENCE, WINK_PULSE_DUR

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

# ── Blink detection tuning ────────────────────────────────────────────────────
RISE_THRESH = 144   # right eye (AF8) — raised +20% (was 120) to need a stronger blink
MIN_PEAK    = 216   # right eye (AF8) — raised +20% (was 180)
FALL_FRAC   = 0.40
COOLDOWN    = 0.20   # lowered so a deliberate double-blink registers two spikes
SHOW_MS     = 500
# Left eye (AF7) is harder to pick up, so give it a lower threshold (−15% from before).
LEFT_RISE_THRESH = 77    # was 90 (−15%)
LEFT_MIN_PEAK    = 115   # was 135 (−15%)

# ── Blink → command timing ────────────────────────────────────────────────────
BLINK_MERGE   = 0.18   # s  L+R of one physical blink merge into a single event
DOUBLE_WINDOW = 0.45   # s  two blink events within this window = DOUBLE → STOP

# ── Jaw clench detection (drift-proof per-channel TP9/TP10 EMG, from jaw test) ──
JAW_WIN        = 64     # samples (~0.25 s) of TP9/TP10 history
JAW_K          = 4.5    # jump must exceed baseline by K × noise spread (raise = stricter); raised from 3.5 (was firing at rest)
JAW_SPREAD_MIN = 0.08   # spread floor as a fraction of baseline (keeps detection relative)
JAW_HIST_LEN   = 240    # samples of resting history per channel (~ a few seconds)
JAW_COOLDOWN   = 0.8    # s  minimum gap between counted jaw triggers
JAW_CONFIRM_DELAY = 0.25  # s  hold a detected clench this long; if a blink lands in the
                          #    window it was a blink bleeding into TP9/TP10 → FORWARD, not back

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


# ── Blink detector ────────────────────────────────────────────────────────────
class BlinkDetector:
    def __init__(self, rise_thresh: float = RISE_THRESH, min_peak: float = MIN_PEAK):
        self.rise_thresh  = rise_thresh   # per-eye thresholds (left eye is lower)
        self.min_peak     = min_peak
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
            if (val - self.baseline) > self.rise_thresh and val > self.min_peak:
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
                if self.peak > self.min_peak and (now - self.last_blink) > COOLDOWN:
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
        if (not calibrating
                and emg9_over and emg10_over
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
left_det   = BlinkDetector(LEFT_RISE_THRESH, LEFT_MIN_PEAK)   # AF7 — lower threshold
right_det  = BlinkDetector()                                  # AF8 — default threshold
jaw_det    = JawDetector()
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]
_pkt_count = 0
running    = True

# Drive state — 0=STOP  1=FORWARD  2=BACKWARD
_drive_state = 0

# Wink / eye classifier state ─────────────────────────────────────────────────
# Raw detector fires (set in on_sensor, classified in _classify_eye_event)
_raw_left_fire  = False   # AF7 fired this round
_raw_right_fire = False   # AF8 fired this round
_raw_left_ts    = 0.0     # timestamp of most recent AF7 fire
_raw_right_ts   = 0.0     # timestamp of most recent AF8 fire

# Active wink steering pulse: None or ('L'|'R', expiry_time)
_wink_pulse: tuple | None = None

# Most-recent eye event for the dashboard ('LEFT'|'RIGHT'|'BOTH'|None)
_last_eye_event: str | None = None

# Both-eye blink event resolution (single = FORWARD, double = STOP)
_blink_last_event   = 0.0
_blink_pending      = False
_blink_pending_time = 0.0
_blink_double_flag  = False

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


# ── Eye-event classifier (called from Muse notification thread) ───────────────
def _register_eye_fires(blink_l: bool, blink_r: bool, now: float) -> None:
    """Record raw detector fires; classification happens in _classify_eye_event."""
    global _raw_left_fire, _raw_right_fire, _raw_left_ts, _raw_right_ts
    if blink_l:
        _raw_left_fire = True
        _raw_left_ts   = now
    if blink_r:
        _raw_right_fire = True
        _raw_right_ts   = now


def _start_wink(side: str, now: float) -> None:
    """A single-eye wink → a brief curved-turn pulse. Only arm the pulse while the
    car is moving; a wink while stopped is shown but does nothing (blink to go first)."""
    global _wink_pulse, _last_eye_event
    _last_eye_event = 'LEFT' if side == 'L' else 'RIGHT'
    if _drive_state != 0:
        _wink_pulse = (side, now + WINK_PULSE_DUR)


def _classify_eye_event() -> None:
    """Run in control loop (~20 Hz). Turn raw AF7/AF8 fires into LEFT / RIGHT / BOTH.

    Rule: if BOTH eyes fire within WINK_COINCIDENCE it is a normal blink — never a
    wink. Only a single eye firing with no partner inside the window is a wink. This
    stops an asymmetric blink (eyes lagging) being mis-split into a left+right pair."""
    global _raw_left_fire, _raw_right_fire, _last_eye_event
    now = time.time()

    with _lock:
        l_pending = _raw_left_fire
        r_pending = _raw_right_fire
        l_ts      = _raw_left_ts
        r_ts      = _raw_right_ts

    if not l_pending and not r_pending:
        return

    # Both eyes fired → normal blink. Commit once the EARLIER fire ages past the
    # window (so any small inter-eye lag has been absorbed).
    if l_pending and r_pending:
        if (now - min(l_ts, r_ts)) >= WINK_COINCIDENCE:
            with _lock:
                _raw_left_fire  = False
                _raw_right_fire = False
            _last_eye_event = 'BOTH'
            _register_blink(now)
        return

    # Only one eye so far — wait WINK_COINCIDENCE for the partner before committing
    # to a wink. If the partner fires meanwhile, the branch above reclassifies it.
    if l_pending and (now - l_ts) >= WINK_COINCIDENCE:
        with _lock:
            _raw_left_fire = False
        _start_wink('L', now)
    elif r_pending and (now - r_ts) >= WINK_COINCIDENCE:
        with _lock:
            _raw_right_fire = False
        _start_wink('R', now)


def _register_blink(now: float) -> None:
    """A both-eye blink is one blink event (single → FORWARD, double → STOP). It
    also cancels a just-detected clench: a blink bleeds into TP9/TP10, so a clench
    landing within JAW_CONFIRM_DELAY of a blink was really that blink → FORWARD."""
    global _blink_last_event, _blink_pending, _blink_pending_time, _blink_double_flag
    global _last_blink_fire, _jaw_pending
    if now - _blink_last_event < BLINK_MERGE:
        return   # duplicate fire from the same physical blink
    _blink_last_event = now
    _last_blink_fire  = now
    if _jaw_pending and (now - _jaw_pending_time) < JAW_CONFIRM_DELAY:
        _jaw_pending = False             # that "clench" was a blink bleeding into TP9/TP10
    if _blink_pending and (now - _blink_pending_time) <= DOUBLE_WINDOW:
        _blink_pending     = False
        _blink_double_flag = True        # second blink → DOUBLE → STOP
    else:
        _blink_pending      = True        # first blink → wait for a possible second
        _blink_pending_time = now


def _resolve_blinks() -> None:
    """Run in control loop. A pending single blink is deferred until DOUBLE_WINDOW
    closes (so it can't fire FORWARD just before a STOP double-blink). Returns
    (single, double) so update_control can apply them with STOP winning ties."""
    global _blink_pending, _blink_double_flag
    now = time.time()
    with _lock:
        double = _blink_double_flag
        _blink_double_flag = False
        single = False
        if _blink_pending and (now - _blink_pending_time) > DOUBLE_WINDOW:
            _blink_pending = False
            single = True
    return single, double


# ── Sensor callback ───────────────────────────────────────────────────────────
def on_sensor(handle, data: bytearray):
    global _pkt_count, _jaw_pending, _jaw_pending_time

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
                    _register_eye_fires(blink_l, blink_r, now)
            # A clench arms a pending BACKWARD; it commits only after
            # JAW_CONFIRM_DELAY with no blink (a coincident blink → it was bleed → FWD).
            if jaw_fired and not _is_syncing():
                with _lock:
                    _jaw_pending      = True
                    _jaw_pending_time = now
        # ACCGYRO (tag 0x47) is ignored — steering is wink-based.


def on_ctrl(handle, data: bytearray):
    pass


# ── Control logic (called at 20 Hz from UI thread) ────────────────────────────
def update_control() -> None:
    global _drive_state, _jaw_pending, _wink_pulse

    _classify_eye_event()           # raw AF7/AF8 fires → LEFT / RIGHT / BOTH
    single, double = _resolve_blinks()

    now = time.time()
    # Commit a pending clench to BACKWARD once it has aged JAW_CONFIRM_DELAY with no
    # blink. A blink just before the clench (bleed) also cancels it (handled below);
    # a blink just after cancels it in _register_blink.
    jaw_back = False
    with _lock:
        if _jaw_pending and (now - _jaw_pending_time) >= JAW_CONFIRM_DELAY:
            _jaw_pending = False
            if _jaw_pending_time - _last_blink_fire >= JAW_CONFIRM_DELAY:
                jaw_back = True

    # Apply latched drive commands. Order matters: STOP (double-blink) wins ties.
    if jaw_back:
        _drive_state = 2          # BACKWARD
    if single:
        _drive_state = 1          # FORWARD
    if double:
        _drive_state = 0          # STOP

    # Safety: never drive unless the Muse is connected and the sync is done
    if not _muse_connected or _is_syncing():
        send_cmd('S')
        return

    now = time.time()

    # Wink steering (brief pulse): a left/right wink curves for WINK_PULSE_DUR.
    if _wink_pulse is not None and now >= _wink_pulse[1]:
        _wink_pulse = None
    left  = _wink_pulse is not None and _wink_pulse[0] == 'L'
    right = _wink_pulse is not None and _wink_pulse[0] == 'R'

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


def _wink_str() -> str:
    """Wink-steering indicator for the dashboard."""
    now   = time.time()
    pulse = _wink_pulse
    if pulse is not None and now < pulse[1]:
        rem = pulse[1] - now
        if pulse[0] == 'L':
            return f'\033[1;96m◄ LEFT WINK\033[0m \033[90m({rem:.1f}s)\033[0m'
        return f'\033[1;96mRIGHT WINK ►\033[0m \033[90m({rem:.1f}s)\033[0m'
    if _last_eye_event == 'LEFT':
        return '\033[96m◄ left wink\033[0m'
    if _last_eye_event == 'RIGHT':
        return '\033[96mright wink ►\033[0m'
    return '\033[92mSTRAIGHT\033[0m'


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
        f'  Steer:  {_wink_str()}',
        f'',
        f'  \033[96m── EEG ─────────────────────────────────────────────────────\033[0m',
        f'  TP9   {eeg[0]:7.1f} µV   {_eeg_bar(eeg[0])}',
        f'  AF7   {lv:7.1f} µV   {_eeg_bar(lv)}  {_blink_tag(left_det)} L:{left_det.count}  base:{lb:.0f}',
        f'  AF8   {rv:7.1f} µV   {_eeg_bar(rv)}  {_blink_tag(right_det)} R:{right_det.count}  base:{rb:.0f}',
        f'  TP10  {eeg[3]:7.1f} µV   {_eeg_bar(eeg[3])}',
        f'',
        f'  \033[96m── JAW ─────────────────────────────────────────────────────\033[0m',
        f'  {_jaw_str()}',
        f'',
        f'  CMD: {_CMD_DESC.get(_last_cmd, "---")}',
        f'  \033[90mCtrl-C quit · blink=FWD · jaw=BACK · 2 blinks=STOP · L/R wink=turn\033[0m',
    ]
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()


def _ui_thread() -> None:
    """Dashboard + control loop. Runs the whole program lifetime so connection
    status is visible even while the Muse/HC-08 are still connecting."""
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

                # Start the 30s sync only on the first successful connect
                if _sync_start == float('inf'):
                    _sync_start = time.time()

                _muse_connected = True
                _muse_status    = f'Streaming  {MUSE_ADDR}'

                # Keep the connection alive until it drops or we quit
                while client.is_connected and running:
                    await asyncio.sleep(0.2)

        except Exception as e:
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
    time.sleep(0.3)
    print("\nStopped.")


if __name__ == "__main__":
    main()
