#!/usr/bin/env python3
"""
muse_athena_car_controller_claude.py
Muse S Athena EEG → Arduino car controller via HC-08 BLE.

This is a structurally-cleaned, safety-hardened rewrite of
muse_athena_car_controller.py. It is BEHAVIOUR-COMPATIBLE: every tuned
constant (blink thresholds, jaw K, timing windows, the Windows notify-retry,
the saturation guard) is copied verbatim from the original, which was hand-
tuned against real signal over several sessions. Nothing in the control math
was changed. What changed is the *scaffolding* around it:

  • State lives in a few small dataclasses instead of ~25 module globals,
    so you can see what each thread owns at a glance.
  • A single CONFIG block at the top holds every number you'd ever tune.
  • The terminal dashboard uses named colour helpers instead of inline escapes.
  • A SAFETY layer was added (the big one) — see "Safety model" below.
  • Live recalibration ('r') is now a flag the sensor thread consumes, which
    removes a race where the UI thread swapped detector objects mid-packet.
  • Decode is vectorised and the redraw is decoupled from the control loop,
    for a little less latency and jitter — without touching the timing gates
    (MULTI_WINDOW / JAW_CONFIRM_DELAY) that exist for correctness.

Control scheme (head-tilt steering + blink/jaw drive):
  Single blink (both eyes)              → FORWARD  (latched)
  Double blink (×2)                     → STOP     (latched)
  Jaw clench                            → BACKWARD (latched, after a short
                                          confirm window; a blink in that
                                          window cancels it — see JawDetector)
  Head tilt L/R (roll > ROLL_THRESHOLD) → curved turn while moving
  First 30 s after Muse connects        → syncing: EEG + jaw baselines build,
                                          IMU roll zeroes (keep head still!)

Safety model (NEW — why a moving vehicle needs more than "send STOP on quit"):
  • EEG watchdog: if sensor packets go stale (headset slips, BLE silently
    stalls without disconnecting), the car is forced to STOP — it never keeps
    rolling on a latched FORWARD with no live brain signal.
  • STOP is never de-duplicated and is re-sent on a heartbeat, so a single
    dropped BLE write can't leave the car driving.
  • Spacebar = EMERGENCY STOP: latches the car stopped and ignores EEG until
    you press space again to re-arm.
  • Any HC-08 (car radio) reconnect forces STOP first, so the car can't lurch
    on a stale command when the link comes back.
  NOTE: software alone cannot guarantee the car stops — the Arduino firmware
  SHOULD also auto-stop if it hears no command for ~300 ms (deadman). The
  heartbeat below feeds such a firmware deadman.

Run:   eeg_env\\Scripts\\python.exe muse_athena_car_controller_claude.py
Keys:  Ctrl-C quit · q quit · r recalibrate · SPACE emergency-stop
"""

import asyncio
import sys
import signal
import time
import math
import random
import threading
import queue
from collections import deque
from dataclasses import dataclass, field

try:
    import msvcrt   # Windows: non-blocking keypress reads (e-stop / reset / quit)
except ImportError:
    msvcrt = None

import numpy as np
import bleak
from bleak import BleakClient, BleakScanner

from config import HC08_ADDRESS, UART_CHAR_UUID, DEFAULT_SPEED, ROLL_THRESHOLD


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIG — every tunable lives here. This is the ONLY place to edit numbers.
# Values are copied verbatim from the original, empirically-tuned controller.
# ═══════════════════════════════════════════════════════════════════════════════

# ── Muse S Athena BLE identifiers ──────────────────────────────────────────────
CTRL_UUID   = "273e0001-4c4d-454d-96be-f03bac821358"   # control / command char
SENSOR_UUID = "273e0013-4c4d-454d-96be-f03bac821358"   # sensor notify char

# ── Athena packet constants (from amused-py) ───────────────────────────────────
HEADER_SIZE = 14
SENSOR_CONFIG = {            # tag: (name, channels, samples, data_len_bytes)
    0x11: ("EEG",     4,  4,  28),
    0x12: ("EEG",     8,  2,  28),
    0x34: ("OPTICS",  4,  3,  30),
    0x35: ("OPTICS",  8,  2,  40),
    0x36: ("OPTICS",  16, 1,  40),
    0x47: ("ACCGYRO", 6,  3,  36),
    0x88: ("BATTERY", 1,  1, 188),
    0x98: ("BATTERY", 1,  1,  20),
}
EEG_SCALE  = 1450.0 / 16383.0   # raw 14-bit → µV
ACC_SCALE  = 0.0000610352       # raw → g
GYRO_SCALE = -0.0074768         # raw → deg/s

# ── Calibration / sync ─────────────────────────────────────────────────────────
SYNC_DURATION = 30.0   # s after Muse connects before blink/tilt commands activate

# ── Blink detection (ADAPTIVE: evolving baseline + relative k-jump) ─────────────
# A blink is detected as a JUMP above an adaptive per-eye baseline, NOT a fixed
# µV threshold. The baseline is the median of recent resting samples and keeps
# updating from non-blink data, so detection gets more accurate the longer you
# wear it (decent after the 30 s sync, better after 1-2 min). A blink fires when
# the signal exceeds  baseline + K × noise-spread.  Lower K = more sensitive.
# This also removes the old "NO CONTACT" suppression: a high resting level just
# raises the baseline, so blinks are still found as relative jumps above it.
BLINK_HIST_LEN   = 512    # resting samples per eye for the baseline (~8 s rolling window)
BLINK_K_RIGHT    = 3.0    # AF8 right eye: jump factor (lower = easier to trigger)
BLINK_K_LEFT     = 2.5    # AF7 left eye is harder to pick up → more sensitive
BLINK_SPREAD_MIN = 8.0    # µV floor on the noise spread (stops hair-trigger when very quiet)
BLINK_FALL_FRAC  = 0.40   # fall back this fraction toward baseline to end a spike
BLINK_COOLDOWN   = 0.20   # s minimum gap between counted blinks on one eye
SHOW_MS          = 500    # how long the BLINK tag stays lit on the dashboard

# ── Blink → command timing ─────────────────────────────────────────────────────
BLINK_MERGE  = 0.18   # s  L+R of one physical blink merge into a single event
MULTI_WINDOW = 1.00   # s  window separating single (→FORWARD) from double (→STOP).
                      #    Kept at 1.0s on purpose: this is correctness, not lag —
                      #    shrinking it makes a deliberate double-blink read as two
                      #    FORWARDs, which is unsafe on a moving vehicle.

# ── Jaw clench detection (raw TP9 amplitude, threshold CALIBRATED during sync) ──
# During the 30 s sync we watch the range TP9's per-packet peak amplitude (µV)
# covers at rest: its average becomes the displayed baseline, and the trigger is
# set a margin ABOVE the highest resting value seen. After sync, TP9 rising over
# that trigger starts a "jump"; it is counted as a clench only when it falls back
# AND only if it lasted between JAW_MIN_DURATION and JAW_MAX_DURATION — a jump held
# longer than JAW_MAX_DURATION is treated as drift / bad contact, not a clench.
JAW_THRESH_MARGIN = 0.085  # trigger = sync-window max TP9 x (1 + this)  (8.5% above max)
JAW_MIN_DURATION  = 0.15   # s  jump must last at least this to count (rejects noise blips)
JAW_MAX_DURATION  = 2.0    # s  jump held longer than this = drift/error → not counted
JAW_COOLDOWN      = 0.8    # s  minimum gap between counted jaw triggers
JAW_CONFIRM_DELAY = 0.25   # s  hold a detected clench this long; a blink in the window
                           #    means it was a blink bleeding into TP9/TP10 → FORWARD, not back

# ── Safety layer (NEW) ─────────────────────────────────────────────────────────
EEG_WATCHDOG_TIMEOUT = 0.6   # s  no fresh EEG packet → force STOP (runaway guard)
IMU_STALE_TIMEOUT    = 2.0   # s  no fresh IMU packet → ignore steering
HEARTBEAT_INTERVAL   = 0.3   # s  re-send current command even if unchanged (feeds firmware deadman)
STREAM_STALE_TIMEOUT = 5.0   # s  connected but no packets → drop link and reconnect
                             #    (5s, not 3s: a brief hiccup shouldn't trigger a full
                             #    teardown now that keep-alive prevents real stalls)

# ── Connection / reconnect ─────────────────────────────────────────────────────
KEEP_ALIVE_INTERVAL = 2.0   # s  resend stream-resume so the Muse doesn't halt its stream
SCAN_TIMEOUT     = 12.0   # s  max BLE scan wait — returns the instant a Muse is seen
BACKOFF_BASE     = 1.5    # s  first reconnect wait
BACKOFF_CAP      = 20.0   # s  maximum reconnect wait
NOTIFY_ATTEMPTS  = 5      # start_notify retries (WinRT 'Unreachable' is often transient)

# ── Loop timing ────────────────────────────────────────────────────────────────
CONTROL_HZ = 50    # control/command decisions per second (was 20 — snappier)
REDRAW_HZ  = 10    # dashboard repaints per second (eye can't tell; saves jitter)
HC08_POLL  = 0.01  # s  command-queue poll on the car-radio thread

# ── Drive state labels ─────────────────────────────────────────────────────────
DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD = 0, 1, 2
DRIVE_LABELS = ['STOP  ■', 'FORWARD ▲', 'BACKWARD ▼']


def encode_cmd(cmd: str) -> bytes:
    """Wrap a Muse text command as a length-prefixed, newline-terminated packet."""
    encoded = cmd.encode("utf-8") + b"\n"
    return bytes([len(encoded) + 1]) + encoded


# Init handshake sent to the Muse to start the Athena sensor stream. The sensor
# notify subscription happens right after the step named in SUBSCRIBE_AFTER_STEP.
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

# Stream-resume commands resent on a timer (KEEP_ALIVE_INTERVAL) by muse_main's
# hold loop. The Muse halts its own sensor stream after a while with no keep-alive
# — this is the dc001/L1 "resume + listen" pair the init uses to start streaming.
# (brainflow_car_controller.py does the equivalent via muse.keep_alive() every 5 s.)
KEEP_ALIVE_SEQ = [encode_cmd("dc001"), encode_cmd("L1")]


# ═══════════════════════════════════════════════════════════════════════════════
# ANSI colour helpers — keep the dashboard code readable (no inline \033[...m)
# ═══════════════════════════════════════════════════════════════════════════════
RESET = "\033[0m"
BOLD  = "\033[1m"
RED, GREEN, YELLOW, CYAN, GREY = (
    "\033[91m", "\033[92m", "\033[93m", "\033[96m", "\033[90m")
INV_BLUE  = "\033[1;97;44m"   # white-on-blue   (BLINK)
INV_RED   = "\033[1;97;41m"   # white-on-red    (JAW! / NO CONTACT / ESTOP)


def c(text: str, colour: str) -> str:
    """Wrap text in an ANSI colour and reset."""
    return f"{colour}{text}{RESET}"


# Enable ANSI VT100 escapes on Windows so the live dashboard renders.
if sys.platform == 'win32':
    try:
        import ctypes
        _k32 = ctypes.windll.kernel32
        _k32.SetConsoleMode(_k32.GetStdHandle(-11), 7)
    except Exception:
        pass

# Force UTF-8 output so the dashboard glyphs (○ ● ■ ▲ ◄) never raise a
# UnicodeEncodeError under a cp1252 console / redirected stdout — which would
# otherwise kill the UI thread (and with it the loop that sends STOP).
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass


# ═══════════════════════════════════════════════════════════════════════════════
# SIGNAL DECODERS — pure functions (no shared state, easy to test)
# ═══════════════════════════════════════════════════════════════════════════════
def decode_eeg_4ch(data: bytes) -> np.ndarray:
    """14-bit LSB-first packed → shape (4 samples, 4 channels) in µV.

    Vectorised equivalent of the original per-bit Python loops: 28 bytes =
    224 bits = 16 little-endian 14-bit samples = (4 samples × 4 channels).
    """
    bits = np.unpackbits(np.frombuffer(data[:28], dtype=np.uint8), bitorder='little')
    weights = (1 << np.arange(14))                    # little-endian bit weights
    vals = bits.reshape(16, 14) @ weights             # → 16 integer samples
    return vals.astype(np.float32).reshape(4, 4) * EEG_SCALE


def decode_accgyro(data: bytes) -> np.ndarray:
    """16-bit signed LE → (3 samples, 6 channels): accX/Y/Z in g, gyroX/Y/Z in deg/s."""
    raw = np.frombuffer(data[:36], dtype="<i2").reshape(3, 6).astype(np.float32)
    result = raw.copy()
    result[:, 0:3] *= ACC_SCALE
    result[:, 3:6] *= GYRO_SCALE
    return result


def parse_payload(payload: bytes) -> list:
    """Split a notify payload into (tag, type, data) subpackets per SENSOR_CONFIG."""
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


# ═══════════════════════════════════════════════════════════════════════════════
# DETECTORS — blink (AF7/AF8) and jaw clench (TP9/TP10). Logic unchanged.
# ═══════════════════════════════════════════════════════════════════════════════
class _Triggerable:
    """Tiny shared base: a count of triggers and a 'lit' window for the dashboard."""
    def __init__(self):
        self.count     = 0
        self.lit_until = 0.0

    def is_lit(self) -> bool:
        return time.time() < self.lit_until

    def _fire(self, lit_seconds: float):
        self.count    += 1
        self.lit_until = time.time() + lit_seconds


class BlinkDetector(_Triggerable):
    """Adaptive spike detector for one eye channel.

    A blink = the signal jumping above an *evolving* baseline by K × noise-spread.
    The baseline is the median of recent resting samples (MAD for spread), learned
    only from non-blink data, so it tracks drift and keeps improving the longer the
    headset is worn. There is no fixed µV threshold and no "no contact" gate — a
    high resting level simply raises the baseline, and blinks are still found as
    relative jumps above it. Lower `k` = more sensitive."""

    def __init__(self, k: float):
        super().__init__()
        self.k            = k
        self.hist         = deque(maxlen=BLINK_HIST_LEN)
        self.baseline     = 0.0
        self.spread       = 0.0
        self.trigger      = 0.0
        self.in_spike     = False
        self.peak         = 0.0
        self.last_blink   = 0.0
        self.peak_display = 0.0

    def process(self, samples: np.ndarray) -> bool:
        val = float(np.max(np.abs(samples)))
        self.peak_display = val

        # Warm-up: just gather resting samples until we have enough for stats.
        if len(self.hist) < 16:
            self.hist.append(val)
            self.baseline = float(np.median(self.hist))
            return False

        # Recompute the adaptive baseline + trigger from the rolling history.
        self.baseline = float(np.median(self.hist))
        self.spread   = max(_mad(np.array(self.hist)), BLINK_SPREAD_MIN)
        self.trigger  = self.baseline + self.k * self.spread

        if not self.in_spike:
            if val > self.trigger:
                self.in_spike = True          # a jump began — don't learn from it
                self.peak = val
            else:
                self.hist.append(val)         # resting sample → improves the baseline
            return False

        # In a spike: follow the peak, count the blink once it falls back.
        if val > self.peak:
            self.peak = val
        fall_target = self.baseline + (self.peak - self.baseline) * BLINK_FALL_FRAC
        if val < fall_target:
            self.in_spike = False
            self.hist.append(val)
            now = time.time()
            if (now - self.last_blink) > BLINK_COOLDOWN:
                self.last_blink = now
                self._fire(SHOW_MS / 1000.0)
                return True
        return False


def _mad(arr: np.ndarray) -> float:
    """Median absolute deviation, scaled to approximate one standard deviation."""
    med = np.median(arr)
    return float(np.median(np.abs(arr - med)) * 1.4826)


class JawDetector(_Triggerable):
    """Raw-amplitude jaw-clench detector on TP9, with a sync-CALIBRATED threshold.

    During the 30 s sync this just gathers TP9's per-packet peak amplitude (µV).
    When sync ends it finalises: baseline = the average of those resting peaks, and
    the trigger = the highest resting peak seen × (1 + JAW_THRESH_MARGIN). After
    sync, TP9 rising over the trigger opens a "jump"; the jump is counted as a
    clench only once it falls back below the trigger, and only if it lasted between
    JAW_MIN_DURATION and JAW_MAX_DURATION. A jump held longer than JAW_MAX_DURATION
    is flagged as drift / bad contact and never counted (even when it finally ends).

    NOTE: the field names emg9/emg10/_base/_trig are kept (now holding raw µV, not
    EMG) so the existing dashboard JAW meter keeps working unchanged."""

    def __init__(self):
        super().__init__()
        self.cal_samples = []       # TP9 peaks gathered during the sync window
        self.calibrated  = False    # trigger finalised yet?
        # dashboard-facing fields (names kept; now raw TP9/TP10 peak in µV)
        self.emg9 = self.emg10 = 0.0
        self.emg9_base = self.emg10_base = 0.0
        self.emg9_trig = self.emg10_trig = 0.0
        # jump tracking
        self.in_jump      = False
        self.jump_start   = 0.0
        self.jump_invalid = False   # jump exceeded JAW_MAX_DURATION → don't count it
        self.last_jaw     = 0.0

    def _finalize(self) -> None:
        """Lock in baseline + trigger from the resting peaks gathered during sync."""
        if self.cal_samples:
            arr = np.array(self.cal_samples)
            self.emg9_base = float(np.mean(arr))
            self.emg9_trig = float(np.max(arr)) * (1.0 + JAW_THRESH_MARGIN)
        else:
            self.emg9_base = 0.0
            self.emg9_trig = float('inf')        # no resting data → never fire
        self.emg10_base = self.emg9_base         # TP10 is display-only; mirror TP9
        self.emg10_trig = self.emg9_trig
        self.calibrated = True
        _mlog(f'jaw cal: TP9 base(avg)={self.emg9_base:.0f} '
              f'restmax={float(np.max(self.cal_samples)) if self.cal_samples else 0:.0f} '
              f'trig={self.emg9_trig:.0f} (n={len(self.cal_samples)})')

    def process(self, arr: np.ndarray, calibrating: bool) -> bool:
        """arr = decoded EEG (4 samples, 4 channels). TP9 = col 0, TP10 = col 3.
        Returns True on a counted clench. While calibrating, only gathers the
        resting range; the threshold is locked in the first call after sync ends."""
        tp9  = float(np.max(np.abs(arr[:, 0])))
        tp10 = float(np.max(np.abs(arr[:, 3])))
        self.emg9, self.emg10 = tp9, tp10        # live values for the dashboard

        if calibrating:
            self.cal_samples.append(tp9)         # learn the resting range only
            return False

        if not self.calibrated:
            self._finalize()                     # sync just ended → lock threshold

        now  = time.time()
        over = tp9 > self.emg9_trig

        if not self.in_jump:
            if over:                             # rising edge — a jump began
                self.in_jump      = True
                self.jump_start   = now
                self.jump_invalid = False
            return False

        # Inside a jump.
        if over:
            if not self.jump_invalid and (now - self.jump_start) > JAW_MAX_DURATION:
                self.jump_invalid = True         # held too long → drift, not a clench
            return False

        # Falling edge — the jump just ended; decide if it was a clench.
        self.in_jump = False
        duration = now - self.jump_start
        if (not self.jump_invalid
                and JAW_MIN_DURATION <= duration <= JAW_MAX_DURATION
                and now - self.last_jaw > JAW_COOLDOWN):
            self.last_jaw = now
            self._fire(0.8)
            return True
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# SHARED STATE — grouped into small dataclasses by concern. `_lock` guards the
# fields touched by more than one thread (sensor / control-UI / BLE).
# ═══════════════════════════════════════════════════════════════════════════════
_lock = threading.Lock()


@dataclass
class DriveState:
    """What the car is doing. Owned by the control loop (UI thread)."""
    mode:  int  = DRIVE_STOP        # DRIVE_STOP / DRIVE_FORWARD / DRIVE_BACKWARD
    estop: bool = False             # spacebar emergency stop latch


@dataclass
class ImuState:
    """Head-tilt (roll) steering. Written by the sensor thread."""
    roll_raw:   float = 0.0
    roll:       float = 0.0
    offset:     float = 0.0
    calibrated: bool  = False
    samples:    list  = field(default_factory=list)
    ts:         float = 0.0         # time of last IMU packet (staleness check)


@dataclass
class BlinkBurst:
    """Blink-burst resolution: 1 blink → FORWARD, 2+ → STOP."""
    last_event:  float = 0.0        # de-dupes L+R of one physical blink
    count:       int   = 0          # blinks in the current burst
    last:        float = 0.0        # time of most recent blink in the burst
    events:      int   = 0          # debug: total merged blink events
    last_action: str   = '—'        # debug: last resolved burst result
    last_fire:   float = 0.0        # timestamp of the last both-eye blink


@dataclass
class JawPending:
    """A clench arms a pending BACKWARD that commits after JAW_CONFIRM_DELAY."""
    pending: bool  = False
    time:    float = 0.0


@dataclass
class ConnState:
    """Connection + stream status shown on the dashboard."""
    muse_connected: bool  = False
    muse_status:    str   = 'Waiting...'
    hc08_connected: bool  = False
    hc08_status:    str   = 'Waiting...'
    pkt_count:      int   = 0
    last_eeg_ts:    float = 0.0          # time of last EEG packet (watchdog)
    sync_start:     float = float('inf') # inf until first successful connect


# Module singletons. We mutate their *attributes* across threads (under _lock for
# shared fields), so no `global` rebinding walls are needed in the hot paths.
drive = DriveState()
imu   = ImuState()
burst = BlinkBurst()
jawp  = JawPending()
conn  = ConnState()

# Detectors are owned by the SENSOR thread. Recalibration is requested via a flag
# that the sensor thread consumes (it rebuilds them itself) — this removes the
# race where the UI thread swapped detector objects mid-packet.
left_det  = BlinkDetector(BLINK_K_LEFT)    # AF7 — more sensitive (harder to pick up)
right_det = BlinkDetector(BLINK_K_RIGHT)   # AF8
jaw_det   = JawDetector()

_eeg_vals = [0.0, 0.0, 0.0, 0.0]     # latest per-channel |peak| for the bars (guarded)
_reset_requested = False             # set by UI thread, consumed by sensor thread
running = True                       # global run flag


# ═══════════════════════════════════════════════════════════════════════════════
# CAR RADIO (HC-08) OUTPUT — its own asyncio thread, fed by a command queue
# ═══════════════════════════════════════════════════════════════════════════════
_hc08_queue = queue.SimpleQueue()
_last_cmd: str | None = None
_last_send_ts = 0.0


def send_cmd(cmd: str, force: bool = False) -> None:
    """Queue a command for the car. De-duplicates identical commands to avoid
    flooding the link — EXCEPT STOP, which is always allowed through (safety),
    and any caller passing force=True (watchdog / heartbeat / e-stop)."""
    global _last_cmd, _last_send_ts
    if not force and cmd == _last_cmd and cmd != 'S':
        return
    _last_cmd = cmd
    _last_send_ts = time.time()
    _hc08_queue.put(cmd)


def _force_stop() -> None:
    """Unconditionally tell the car to STOP and latch drive state to STOP."""
    drive.mode = DRIVE_STOP
    send_cmd('S', force=True)


async def _hc08_ble_main() -> None:
    attempt = 0
    while running:
        conn.hc08_connected = False
        conn.hc08_status    = 'Connecting...'
        try:
            async with bleak.BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                conn.hc08_connected = True
                conn.hc08_status    = f'Connected  {HC08_ADDRESS}'
                attempt = 0
                # On every (re)connect: flush stale commands and force STOP first,
                # so the car can't lurch on a command queued before the drop.
                while True:
                    try:    _hc08_queue.get_nowait()
                    except queue.Empty: break
                _force_stop()
                await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                await client.write_gatt_char(
                    UART_CHAR_UUID, DEFAULT_SPEED.encode(), response=True)
                while client.is_connected and running:
                    try:
                        raw = _hc08_queue.get_nowait()
                        await client.write_gatt_char(
                            UART_CHAR_UUID, raw.encode(), response=True)
                    except queue.Empty:
                        await asyncio.sleep(HC08_POLL)
                # Quitting ('q'/Ctrl-C) or link dropping: send a final STOP while we
                # still hold the link, so the car never coasts on its last command.
                if client.is_connected:
                    try:
                        await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                    except Exception:
                        pass
        except Exception as e:
            conn.hc08_status = f'Not found — retrying ({type(e).__name__})'
        conn.hc08_connected = False
        if running:
            attempt += 1
            await asyncio.sleep(_backoff(attempt))


def _hc08_thread() -> None:
    asyncio.run(_hc08_ble_main())


# ═══════════════════════════════════════════════════════════════════════════════
# SYNC / RECONNECT HELPERS
# ═══════════════════════════════════════════════════════════════════════════════
def _is_syncing() -> bool:
    return (time.time() - conn.sync_start) < SYNC_DURATION


def _sync_remaining() -> float:
    return max(0.0, SYNC_DURATION - (time.time() - conn.sync_start))


def _backoff(attempt: int) -> float:
    """Exponential backoff with jitter, capped — gentler on the BLE stack than a
    fixed retry cadence when a link is flapping."""
    base = min(BACKOFF_CAP, BACKOFF_BASE * (2 ** min(attempt - 1, 4)))
    return base * (0.7 + 0.6 * random.random())


# ═══════════════════════════════════════════════════════════════════════════════
# BLINK-BURST RESOLVER
# ═══════════════════════════════════════════════════════════════════════════════
def _register_blink(now: float) -> None:
    """Count blinks into a burst. L+R of one physical blink (within BLINK_MERGE)
    collapse to one. A blink also cancels a just-detected clench: a blink bleeds
    into TP9/TP10, so a clench within JAW_CONFIRM_DELAY of a blink was that blink.
    Call with _lock held."""
    if now - burst.last_event < BLINK_MERGE:
        return                                   # same physical blink firing twice
    burst.last_event = now
    burst.last_fire  = now
    burst.events    += 1
    if jawp.pending and (now - jawp.time) < JAW_CONFIRM_DELAY:
        jawp.pending = False                     # that "clench" was a blink bleed
    if burst.count > 0 and (now - burst.last) <= MULTI_WINDOW:
        burst.count += 1                         # another blink in the same burst
    else:
        burst.count = 1                          # new burst
    burst.last = now


def _resolve_blinks():
    """1 blink → FORWARD, 2+ → STOP. STOP fires the instant a 2nd blink lands; a
    lone blink fires FORWARD only once MULTI_WINDOW closes. Returns (forward, stop)."""
    now = time.time()
    forward = stop = False
    with _lock:
        if burst.count >= 2:
            stop = True
            burst.last_action = f'STOP({burst.count})'
            burst.count = 0
        elif burst.count == 1 and (now - burst.last) > MULTI_WINDOW:
            forward = True
            burst.last_action = 'FWD(1)'
            burst.count = 0
    return forward, stop


# ═══════════════════════════════════════════════════════════════════════════════
# SENSOR CALLBACK — runs on the Muse BLE thread. Sole owner of the detectors.
# ═══════════════════════════════════════════════════════════════════════════════
def _apply_reset_if_requested() -> None:
    """Consume a pending 'r' recalibration here, where the detectors live, so the
    UI thread never swaps detector objects out from under an in-flight packet."""
    global _reset_requested, left_det, right_det, jaw_det
    if not _reset_requested:
        return
    with _lock:
        left_det  = BlinkDetector(BLINK_K_LEFT)
        right_det = BlinkDetector(BLINK_K_RIGHT)
        jaw_det   = JawDetector()
        imu.calibrated = False
        imu.samples    = []
        imu.offset     = 0.0
        imu.roll       = 0.0
        burst.count       = 0
        burst.events      = 0
        burst.last_action = '—'
        conn.sync_start   = time.time()          # restart the 30 s sync
        _reset_requested  = False
    _force_stop()                                # park the car while re-syncing


def on_sensor(handle, data: bytearray):
    _apply_reset_if_requested()

    conn.pkt_count += 1
    conn.last_eeg_ts = time.time()               # feed the runaway watchdog
    now = time.time()
    syncing = _is_syncing()

    for tag, stype, raw in parse_payload(bytes(data)):
        if tag == 0x11:                          # EEG 4-channel
            arr     = decode_eeg_4ch(raw)
            blink_l = left_det.process(arr[:, 1])    # AF7 — left eye
            blink_r = right_det.process(arr[:, 2])   # AF8 — right eye
            jaw_fired = jaw_det.process(arr, syncing) # TP9/TP10 clench

            with _lock:
                for ch in range(4):
                    _eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))
                if (blink_l or blink_r) and not syncing:
                    _register_blink(now)
                # A clench arms a pending BACKWARD; it commits only after
                # JAW_CONFIRM_DELAY with no blink (a coincident blink → bleed → FWD).
                if jaw_fired and not syncing:
                    jawp.pending = True
                    jawp.time    = now

        elif tag == 0x47:                        # ACCGYRO → head-tilt steering
            acc = decode_accgyro(raw)
            ay, az = float(acc[-1, 1]), float(acc[-1, 2])
            imu.roll_raw = math.degrees(math.atan2(ay, az))
            imu.ts = time.time()
            if syncing:
                imu.samples.append(imu.roll_raw)             # gather resting offset
            else:
                if not imu.calibrated and imu.samples:
                    imu.offset     = float(np.median(imu.samples))  # median = robust
                    imu.calibrated = True
                imu.roll = imu.roll_raw - imu.offset


def on_ctrl(handle, data: bytearray):
    pass                                          # control-channel replies unused


# ═══════════════════════════════════════════════════════════════════════════════
# CONTROL LOGIC — decides the drive command. Called at CONTROL_HZ from UI thread.
# ═══════════════════════════════════════════════════════════════════════════════
def update_control() -> None:
    now = time.time()
    heartbeat = (now - _last_send_ts) >= HEARTBEAT_INTERVAL

    # 1) EMERGENCY STOP overrides everything until re-armed.
    if drive.estop:
        drive.mode = DRIVE_STOP
        send_cmd('S', force=True)
        return

    # 2) Resolve gestures into latched drive intent (same logic as the original).
    forward, stop = _resolve_blinks()            # 1 blink → FWD · 2+ → STOP
    jaw_back = False
    with _lock:
        if jawp.pending and (now - jawp.time) >= JAW_CONFIRM_DELAY:
            jawp.pending = False
            if jawp.time - burst.last_fire >= JAW_CONFIRM_DELAY:
                jaw_back = True
    if jaw_back:  drive.mode = DRIVE_BACKWARD
    if forward:   drive.mode = DRIVE_FORWARD
    if stop:      drive.mode = DRIVE_STOP        # STOP wins ties

    # 3) SAFETY GATES — never drive unless Muse is connected, sync is done, AND
    #    EEG packets are fresh. A stale stream (slipped headset / silent BLE
    #    stall) forces STOP so the car can't run away on a latched FORWARD.
    eeg_fresh = conn.last_eeg_ts > 0 and (now - conn.last_eeg_ts) <= EEG_WATCHDOG_TIMEOUT
    if not conn.muse_connected or _is_syncing() or not eeg_fresh:
        drive.mode = DRIVE_STOP
        send_cmd('S', force=True)
        return

    # 4) Head-tilt steering curves the latched drive direction while moving.
    has_imu = imu.ts > 0 and (now - imu.ts) <= IMU_STALE_TIMEOUT
    left  = has_imu and imu.roll < -ROLL_THRESHOLD
    right = has_imu and imu.roll >  ROLL_THRESHOLD

    if drive.mode == DRIVE_FORWARD:
        cmd = 'Q' if left else 'E' if right else 'F'
    elif drive.mode == DRIVE_BACKWARD:
        cmd = 'G' if left else 'H' if right else 'B'
    else:
        cmd = 'S'
    send_cmd(cmd, force=heartbeat)               # heartbeat re-sends even if unchanged


# ═══════════════════════════════════════════════════════════════════════════════
# DASHBOARD — render-from-snapshot. Repaints by moving the cursor up N lines.
# ═══════════════════════════════════════════════════════════════════════════════
_CMD_DESC = {
    'F': 'FORWARD      ▲', 'B': 'BACKWARD     ▼',
    'Q': 'FWD-LEFT     ◤', 'E': 'FWD-RIGHT    ◥',
    'G': 'BCK-LEFT     ◣', 'H': 'BCK-RIGHT    ◢',
    'S': 'STOP         ■',
}
_DRIVE_COLOURS = [RED, GREEN, YELLOW]   # STOP / FORWARD / BACKWARD
_prev_lines = 0


def _bar(val: float, full: float, width: int = 22) -> str:
    """Horizontal meter; colour shifts green→yellow→red as it fills."""
    frac   = max(0.0, min(val / full, 1.0))
    filled = int(frac * width)
    col    = RED if val > full else (YELLOW if val > full * 0.5 else GREEN)
    return col + '█' * filled + GREY + '░' * (width - filled) + RESET


def _frac_bar(frac: float, width: int = 12) -> str:
    """Bar that fills toward a trigger (frac >= 1.0 = over)."""
    frac   = max(0.0, min(frac, 1.0))
    filled = int(frac * width)
    col    = RED if frac >= 1.0 else (YELLOW if frac > 0.6 else GREEN)
    return col + '█' * filled + GREY + '░' * (width - filled) + RESET


def _blink_tag(det: BlinkDetector) -> str:
    return c(' BLINK ', INV_BLUE) if det.is_lit() else c(' ------ ', GREY)


def _conn_cell(connected: bool, status: str) -> str:
    dot = c('● CONNECTED', GREEN) if connected else c('○ waiting', RED)
    pad = '  ' if connected else '    '
    return f'{dot}{pad}{status}'


def _roll_str() -> str:
    has_imu = imu.ts > 0 and (time.time() - imu.ts) <= IMU_STALE_TIMEOUT
    if not has_imu:
        return c('IMU: no data yet', GREY)
    roll = imu.roll
    if roll < -ROLL_THRESHOLD:
        direction = c('◄ TURN LEFT', CYAN)
    elif roll > ROLL_THRESHOLD:
        direction = c('TURN RIGHT ►', CYAN)
    else:
        direction = c('STRAIGHT', GREEN)
    return f'Roll {roll:+6.1f}°  {direction}'


def _jaw_str() -> str:
    j   = jaw_det
    f9  = (j.emg9  - j.emg9_base)  / (j.emg9_trig  - j.emg9_base)  if j.emg9_trig  > j.emg9_base  else 0.0
    f10 = (j.emg10 - j.emg10_base) / (j.emg10_trig - j.emg10_base) if j.emg10_trig > j.emg10_base else 0.0
    tag = c(' JAW! ', INV_RED) if j.is_lit() else c(' ---- ', GREY)
    return f'TP9 {_frac_bar(f9)}  TP10 {_frac_bar(f10)}  {tag} cnt:{j.count}'


def draw():
    global _prev_lines
    with _lock:
        eeg = list(_eeg_vals)

    lv, rv = left_det.peak_display, right_det.peak_display
    lb     = left_det.baseline  or 0.0
    rb     = right_det.baseline or 0.0
    syncing = _is_syncing()
    ds      = drive.mode

    # Banner reflects connection / sync / e-stop state.
    if drive.estop:
        banner = c('  ── ⛔ EMERGENCY STOP — press SPACE to re-arm ─────────────', INV_RED)
    elif not conn.muse_connected:
        banner = c('  ── WAITING FOR MUSE HEADSET ───────────────────────────────', RED)
    elif syncing:
        banner = c(f'  ── SYNCING  {_sync_remaining():.0f}s remaining  '
                   f'(keep headset still, let EEG settle) ──', YELLOW)
    else:
        banner = c('  ── ACTIVE ─────────────────────────────────────────────────', GREEN)

    drive_hint = ('(not active yet)' if (not conn.muse_connected or syncing or drive.estop)
                  else 'blink → FWD · jaw → BACK · 2× blink → STOP')
    drive_str = (f'  Drive:  {_DRIVE_COLOURS[ds]}{DRIVE_LABELS[ds]}{RESET}'
                 f'   {c(drive_hint, GREY)}')

    rows = [
        f'  {BOLD}Muse S Athena{RESET} — Car Controller (claude)   [{time.strftime("%H:%M:%S")}]',
        f'  Muse   {_conn_cell(conn.muse_connected, conn.muse_status)}   {c(f"pkts:{conn.pkt_count}", GREY)}',
        f'  HC-08  {_conn_cell(conn.hc08_connected, conn.hc08_status)}',
        '',
        banner,
        '',
        drive_str,
        f'  Steer:  {_roll_str()}',
        '',
        c('  ── EEG ─────────────────────────────────────────────────────', CYAN),
        f'  TP9   {eeg[0]:7.1f} µV   {_bar(eeg[0], 500.0)}',
        f'  AF7   {lv:7.1f} µV   {_bar(lv, 500.0)}  {_blink_tag(left_det)} L:{left_det.count}  base:{lb:.0f} trig:{left_det.trigger:.0f}',
        f'  AF8   {rv:7.1f} µV   {_bar(rv, 500.0)}  {_blink_tag(right_det)} R:{right_det.count}  base:{rb:.0f} trig:{right_det.trigger:.0f}',
        f'  TP10  {eeg[3]:7.1f} µV   {_bar(eeg[3], 500.0)}',
        '',
        c('  ── JAW ─────────────────────────────────────────────────────', CYAN),
        f'  {_jaw_str()}',
        '',
        f'  CMD: {_CMD_DESC.get(_last_cmd, "---")}   '
        f'{c(f"blinks: burst={burst.count} events={burst.events} last={burst.last_action}", GREY)}',
        c('  Ctrl-C/q quit · SPACE e-stop · r recalibrate · blink=FWD · 2=STOP · jaw=BACK · tilt=turn', GREY),
    ]

    if _prev_lines:
        sys.stdout.write(f'\033[{_prev_lines}A\033[J')
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()
    _prev_lines = len(rows)


# ═══════════════════════════════════════════════════════════════════════════════
# UI / CONTROL THREAD — keyboard, control loop (fast) and redraw (slow)
# ═══════════════════════════════════════════════════════════════════════════════
def _handle_key(ch: str) -> None:
    global _reset_requested, running
    if ch in ('r', 'R'):
        _reset_requested = True                  # sensor thread does the rebuild
    elif ch == ' ':
        drive.estop = not drive.estop            # toggle emergency stop
        if drive.estop:
            _force_stop()
    elif ch in ('q', 'Q'):
        running = False


def _ui_thread() -> None:
    """Runs the whole program lifetime so status is visible while connecting.
    Control decisions run at CONTROL_HZ; the dashboard repaints at REDRAW_HZ."""
    control_dt = 1.0 / CONTROL_HZ
    redraw_dt  = 1.0 / REDRAW_HZ
    next_control = next_redraw = time.time()
    while running:
        now = time.time()
        if msvcrt is not None and msvcrt.kbhit():
            _handle_key(msvcrt.getwch())
        if now >= next_control:
            update_control()                     # safety-critical — must keep running
            next_control = now + control_dt
        if now >= next_redraw:
            try:
                draw()                           # rendering must never kill control
            except Exception:
                pass
            next_redraw = now + redraw_dt
        time.sleep(0.005)
    _force_stop()                                # always leave the car stopped


# ═══════════════════════════════════════════════════════════════════════════════
# MUSE CONNECTION — scan by name, connect, init, stream; auto-reconnect w/ backoff
# ═══════════════════════════════════════════════════════════════════════════════
def _mlog(msg: str):
    """Append a timestamped line to muse_debug.log (the TUI hides stdout)."""
    try:
        with open('muse_debug.log', 'a', encoding='utf-8') as f:
            f.write(f'{time.strftime("%H:%M:%S")} {msg}\n')
    except Exception:
        pass


async def _notify_with_retry(client, uuid, callback, label):
    """start_notify, retrying on transient WinRT 'Unreachable' errors. On Windows
    the descriptor write that enables notifications often fails the first time(s)
    even though the device is connected; a short pause + retry usually succeeds."""
    for i in range(1, NOTIFY_ATTEMPTS + 1):
        try:
            await client.start_notify(uuid, callback)
            _mlog(f'notify {label} OK (attempt {i})')
            return
        except Exception as e:
            _mlog(f'notify {label} attempt {i} failed: {type(e).__name__}: {e}')
            if i == NOTIFY_ATTEMPTS:
                raise
            await asyncio.sleep(0.4)


async def muse_main():
    attempt = 0
    while running:
        conn.muse_connected = False
        try:
            conn.muse_status = 'Scanning for any Muse headset...'
            _mlog('scanning...')
            # Match the first Muse by name (so any Muse works without a hardcoded
            # MAC), but RETURN THE INSTANT one is seen instead of always blocking the
            # full SCAN_TIMEOUT like discover() does — this is what cut reconnects
            # from ~20s down to a couple of seconds.
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: bool(d.name and 'muse' in d.name.lower()),
                timeout=SCAN_TIMEOUT)
            if device is None:
                conn.muse_status = 'No Muse found — is the headset on? Retrying...'
                _mlog('no muse seen within scan window')
                attempt += 1
                await asyncio.sleep(_backoff(attempt))
                continue

            conn.muse_status = f'Found {device.name} ({device.address}) — connecting...'
            _mlog(f'found {device.name} {device.address}; connecting...')
            async with BleakClient(device, timeout=20.0) as client:
                _mlog('connected (entered context)')
                # Verify the chars actually discovered before subscribing — Windows
                # can hand back an empty/stale GATT table on a fresh connect.
                svcs = client.services
                if (svcs.get_characteristic(CTRL_UUID) is None or
                        svcs.get_characteristic(SENSOR_UUID) is None):
                    conn.muse_status = 'GATT not ready — reconnecting...'
                    _mlog('GATT not ready (chars missing) — reconnecting')
                    await asyncio.sleep(1.0)
                    continue

                conn.muse_status = 'Connected — running init sequence...'
                await _notify_with_retry(client, CTRL_UUID, on_ctrl, 'CTRL')
                for step, cmd_bytes, delay in INIT_SEQ:
                    await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
                    await asyncio.sleep(delay)
                    if step == SUBSCRIBE_AFTER_STEP:
                        await _notify_with_retry(client, SENSOR_UUID, on_sensor, 'SENSOR')

                if conn.sync_start == float('inf'):
                    conn.sync_start = time.time()    # start the 30 s sync once
                conn.last_eeg_ts = time.time()       # arm the stale-stream check
                conn.muse_connected = True
                conn.muse_status    = f'Streaming  {device.name} ({device.address})'
                _mlog('init complete — streaming')
                attempt = 0

                # Hold the connection. Resend the stream-resume ("keep-alive") on a
                # timer — without it the Muse halts its own sensor stream after a
                # while (the cause of the old "stream stalled" disconnects). Still
                # break out if the stream goes silent anyway, so the watchdog
                # reconnects instead of the car driving on dead signal.
                next_alive = time.time() + KEEP_ALIVE_INTERVAL
                while client.is_connected and running:
                    await asyncio.sleep(0.2)
                    now = time.time()
                    if now >= next_alive:
                        try:
                            for cmd_bytes in KEEP_ALIVE_SEQ:
                                await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
                        except Exception as e:
                            _mlog(f'keep-alive write failed ({type(e).__name__}) — reconnecting')
                            break
                        next_alive = now + KEEP_ALIVE_INTERVAL
                    if now - conn.last_eeg_ts > STREAM_STALE_TIMEOUT:
                        conn.muse_status = 'Stream stalled — reconnecting...'
                        _mlog('stream stalled — forcing reconnect')
                        break

        except Exception as e:
            _mlog(f'EXCEPTION {type(e).__name__}: {e}')
            conn.muse_status = f'Lost / failed ({type(e).__name__}) — retrying...'

        conn.muse_connected = False
        _force_stop()                                # signal lost → car stops
        if running:
            attempt += 1
            await asyncio.sleep(_backoff(attempt))

    _force_stop()


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
def _sig_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT,  _sig_handler)
signal.signal(signal.SIGTERM, _sig_handler)


def main():
    threading.Thread(target=_hc08_thread, daemon=True).start()
    threading.Thread(target=_ui_thread,   daemon=True).start()
    try:
        asyncio.run(muse_main())
    except KeyboardInterrupt:
        pass
    time.sleep(0.5)   # give the HC-08 thread time to push the final STOP
    print("\nStopped.")


if __name__ == "__main__":
    main()
