#!/usr/bin/env python3
"""
muse_controll_fable.py
Muse S Athena EEG → Arduino car controller via HC-08 BLE.  (FABLE build)

Behavior-preserving rewrite of muse_controll_alhpa_stopswitch.py: same control
scheme, same brain-wave gating, same BLE protocol, same dashboard — plus 10
correctness / performance / cleanliness fixes found in a code audit.

CHANGES vs muse_controll_alhpa_stopswitch.py:
  1. Dashboard no longer scrolls: the cursor-reposition amount is now derived
     from the actual number of rendered rows (len(rows)) instead of a separate
     hardcoded DISPLAY_LINES constant that had drifted out of sync with them
     (25 vs. the real 26 rows emitted every frame).
  2. HC-08 reconnect now resets the command-dedupe state and immediately
     re-sends the control loop's current drive command, so a reconnect can't
     strand the car on a stale pre-disconnect command that never gets resent.
  3. A Muse disconnect now forces the car to STOP immediately instead of
     silently resuming the previous direction the instant it reconnects.
  4. New EEG-stall watchdog: if the Muse BLE link stays up but no EEG packet
     has arrived for over a second, the car is auto-STOPped; control resumes
     automatically the moment data starts flowing again (no manual reset).
  5. All interval/cooldown/hold-duration timing now uses time.monotonic()
     instead of time.time(), which is not guaranteed monotonic (an NTP sync,
     DST change or manual clock change could previously corrupt cooldown and
     hold-duration logic). Wall-clock time.time()/time.strftime() is kept only
     for the human-readable on-screen clock and the debug-log timestamps.
  6. Removed dead code: JAW_CONFIRM_DELAY, _jaw_pending_time and
     _last_blink_fire were all write-only / never read. Also corrected a
     stale comment that described the blink scheme as "2 blinks → FORWARD,
     3+ blinks → STOP"; the actually-implemented behavior (matching the
     control-scheme section above) is 1 blink → FORWARD, 2+ blinks → STOP.
  7. Shared state that crosses threads (Muse BLE callback thread, HC-08 BLE
     thread, UI/control thread) is now guarded by a small, consistent set of
     locks instead of ad-hoc/partial locking: `_lock` for detection/drive/IMU
     state, `_conn_lock` for connection status text, `_hc08_lock` for the
     HC-08 command-dedupe state. Critical sections are kept small and never
     span blocking I/O, so this adds no meaningful latency to the 20 Hz
     control loop.
  8. Performance: the 14-bit EEG sample unpacking loop is now vectorized with
     numpy (np.unpackbits + a weighted reduction) instead of a per-sample
     Python bit-shifting loop; the 1 s FFT analysis window is now a
     preallocated numpy ring buffer updated in place instead of a deque that
     was rebuilt into a fresh np.array on every incoming packet; and the
     HC-08 send loop now waits on an asyncio.Event (woken immediately by
     send_cmd, with a bounded timeout safety net) instead of busy-polling at
     50 Hz regardless of whether anything changed.
  9. The 'r'-key reader now drains ALL pending keypresses per control tick
     (`while msvcrt.kbhit(): ...`) instead of only the first one.
  10. Removed a duplicate `now = time.time()` call in update_control — there
      is now a single time.monotonic() call per tick, reused everywhere in
      that tick. Also made the bleak client reference consistently use the
      already-imported `BleakClient` name instead of mixing it with the
      fully-qualified `bleak.BleakClient` in one spot.

────────────────────────────────────────────────────────────────────────────────
muse_controll_alhpa_stopswitch.py (original header, preserved for context)

Identical to muse_controll_alhpa.py in every way EXCEPT one safety rule:

  ── DIRECTION LOCK ─────────────────────────────────────────────────────────────
  The car may NEVER flip directly between FORWARD and BACKWARD — a STOP
  (double-blink) must come in between. So:
    • While going FORWARD, a jaw clench (reverse) is IGNORED → stays FORWARD.
    • While going BACKWARD, a single blink (forward) is IGNORED → stays BACKWARD.
    • A double-blink STOP always works; after a STOP either direction is free.
  Example: jaw → double-blink → blink  =  BACKWARD → STOP → FORWARD.
  This prevents a sudden full reversal (jarring / unsafe on a real wheelchair) —
  the rider must deliberately stop before switching direction.

  ── HEAD-NOD LOCK / HEAD-HOLD UNLOCK ───────────────────────────────────────────
  A forward→back→forward→back head NOD (pitch axis) LOCKS the car: it parks in STOP
  and ignores every drive command (blink / jaw / tilt). To UNLOCK, tilt the head
  LEFT and hold 3 s, then tilt RIGHT and hold 3 s (roll axis, long sustained hold so
  ordinary steering can't trip it). Either nod order (F·B·F·B or B·F·B·F) locks.

────────────────────────────────────────────────────────────────────────────────
muse_controll_alhpa.py
Muse S Athena EEG → Arduino car controller via HC-08 BLE.  (ALPHA / WAVE-GATED build)

Everything in this controller is identical to muse_controller_mix.py — the same
BLE connect/decode, the fixed-µV blink detector (FORWARD/STOP), the Adapt jaw
detector (BACKWARD), the blink↔clench mutual-priority logic, head-tilt steering
and dashboard — PLUS one new idea that makes detections more trustworthy:

  ── BRAIN-WAVE GATING ──────────────────────────────────────────────────────────
  Every candidate detection must be CONFIRMED by the frequency-band content of the
  EEG (Delta/Theta/Alpha/Beta/Gamma relative power, on a 1 s rolling window, same
  numpy-FFT method as muse_athena_waves.py). A detection that the bands don't back
  up is rejected as noise/bleed:

    • BLINK  → accepted only if DELTA relative power ≥ 75% on AF7/AF8.
        A real blink is a huge low-frequency deflection that dumps most of its
        power into Delta, so a genuine blink lights up Delta on the eye channels.
        For a DOUBLE blink (STOP) the Delta gate only needs to pass ONCE: the
        first blink must clear it to open the burst, but the second blink is
        accepted even if its own Delta is below the minimum.
    • JAW CLENCH → accepted only if (BETA ≥ 15%) OR (GAMMA ≥ 20%) on TP9/TP10.
        Clench EMG is high-frequency and spreads power into Beta and Gamma. Either
        band crossing its minimum is enough; both is fine too; if NEITHER reaches
        its minimum the clench is rejected.

  Relative power = that band's share of total 1-44 Hz power for the channel. The
  gate uses the strongest of the two relevant channels (max of AF7/AF8 for Delta,
  max of TP9/TP10 for Beta/Gamma). Until a full 1 s window has been buffered the
  gate passes through (startup is already covered by the 30 s sync).

Control scheme (head-tilt steering + blink/jaw drive):
  Single blink (both eyes)          → FORWARD  (latched)   [wave-gated: Delta]
  Double blink (×2)                 → STOP     (latched)   [wave-gated: Delta]
  Jaw clench                        → BACKWARD (latched)   [wave-gated: Beta/Gamma]
  Head tilt L/R (roll > ROLL_THRESHOLD) → curved turn while moving (Q/E fwd · G/H bck)
  First 30 seconds                  → syncing: EEG + jaw baselines build, IMU roll zeroes

Two BLE devices connect independently and the dashboard shows each one's
status (Muse headset + Arduino HC-08). Either may still be connecting while
the other is live. Both keep retrying automatically if they aren't found.

Run:   eeg_env\\Scripts\\python.exe muse_controll_fable.py
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

# ── Blink detection tuning (FORWARD/STOP — from normal muse_athena_car_controller) ──
RISE_THRESH = 144   # right eye (AF8) — raised +20% (was 120) to need a stronger blink
MIN_PEAK    = 216   # right eye (AF8) — raised +20% (was 180)
FALL_FRAC   = 0.40
COOLDOWN    = 0.20   # lowered so a deliberate double-blink registers two spikes
SHOW_MS     = 500
# Left eye (AF7) is harder to pick up, so give it a lower threshold (−15% from before).
LEFT_RISE_THRESH = 77    # was 90 (−15%)
LEFT_MIN_PEAK    = 115   # was 135 (−15%)
# A railed electrode (no skin contact) sits pinned near the 1450 µV ceiling and its
# saturation noise reads as endless false blinks. At/above this we treat the channel
# as "no contact" and suppress blink detection entirely.
SATURATION = 1430.0

# ── Brain-wave gating (ALPHA build) ────────────────────────────────────────────
# Band-power confirmation for every detection. Relative power = band share of the
# total 1-44 Hz power on a channel, from a 1 s rolling window (numpy FFT, the same
# method as muse_athena_waves.py). A detection must be backed up by the bands or it
# is rejected as noise/bleed.
WAVE_FS          = 256.0          # EEG sample rate (Hz)
WAVE_WIN_SEC     = 1.0            # rolling analysis window (s)
WAVE_WIN_SAMPLES = int(WAVE_FS * WAVE_WIN_SEC)
WAVE_BANDS = [
    ("Delta", 1.0,  4.0),
    ("Theta", 4.0,  8.0),
    ("Alpha", 8.0,  13.0),
    ("Beta",  13.0, 30.0),
    ("Gamma", 30.0, 44.0),
]
WAVE_TOTAL_LO, WAVE_TOTAL_HI = 1.0, 44.0   # denominator range for relative power
# Gate thresholds (relative power, 0-1):
DELTA_BLINK_MIN = 0.75   # blink accepted only if Delta ≥ 75% on AF7/AF8.
                         # For a double-blink the gate only has to pass ONCE — the
                         # first blink opens the burst, a following blink in that
                         # same gated burst is accepted even if its own Delta is low.
BETA_JAW_MIN    = 0.15   # clench accepted if Beta ≥ 15% ...
GAMMA_JAW_MIN   = 0.20   # ... OR Gamma ≥ 20% on TP9/TP10 (either is enough)
# A gate is satisfied if its limit is reached at ANY tick within this rolling
# window — not only at the exact tick the blink/clench fires. This makes a
# borderline detection robust to a momentary dip right at the firing instant.
WAVE_GATE_WINDOW = 0.03  # s (30 ms)

# Precomputed Hann window + PSD normalisation + FFT freq bins for one full window.
_WAVE_HANN     = np.hanning(WAVE_WIN_SAMPLES).astype(np.float64)
_WAVE_WIN_NORM = float(np.sum(_WAVE_HANN ** 2))
_WAVE_FREQS    = np.fft.rfftfreq(WAVE_WIN_SAMPLES, d=1.0 / WAVE_FS)

# ── Blink → command timing ────────────────────────────────────────────────────
BLINK_MERGE  = 0.18   # s  L+R of one physical blink merge into a single event
MULTI_WINDOW = 1.00   # s  window to tell a single blink from a double: 1× → FORWARD
                      #    (fires MULTI_WINDOW after the blink), 2× → STOP (fires at once
                      #    on the 2nd blink). Raise if a double-blink reads as two singles.

# ── Blink ↔ clench mutual priority (MIX-only rule) ─────────────────────────────
# FORWARD (blink) and BACKWARD (clench) suppress each other when they land close
# together, so bleed from one motion can't trigger the other. The rule is
# "first-detected wins, ties go to BACKWARD" — BACKWARD is the dominant command
# (longer window + wins ties), but a clearly-earlier blink can still win FORWARD.
#
#   • BACKWARD window — 600 ms (200 ms before → 400 ms after the clench):
#       A clench blocks FORWARD for JAW_BLINK_PRIORITY_POST after it, and any blink
#       up to JAW_BLINK_PRIORITY_PRE *before* the clench is treated as a tie → the
#       clench wins and that blink loses FORWARD.
#   • FORWARD window — 300 ms after the blink (BLINK_JAW_PRIORITY_POST):
#       A blink blocks BACKWARD for this long, so FORWARD wins only when the blink
#       is clearly first (the clench arrives later than the 200 ms tie-reach).
#
# A suppressed command is NOT cached — it is dropped the instant its window closes,
# so the opposite motion needs a fresh event afterwards. A double-blink STOP always
# wins (safety). BACKWARD always commits immediately when it wins — the car never
# waits for any window. (Note: the FORWARD window has no "before" reach — backward
# priority already claims everything up to 200 ms before a clench, so a blink can
# never cancel an earlier clench.)
JAW_BLINK_PRIORITY_PRE  = 0.200   # s  blink up to 200 ms before a clench → tie → BACKWARD wins
JAW_BLINK_PRIORITY_POST = 0.400   # s  clench blocks FORWARD this long after it
BLINK_JAW_PRIORITY_POST = 0.300   # s  blink blocks BACKWARD this long after it (FORWARD wins)

# ── Jaw clench detection (BACKWARD — from muse_athena_controller_Adapt) ─────────
JAW_WIN        = 64     # samples (~0.25 s) of TP9/TP10 history
JAW_K          = 3.375  # jump must exceed baseline by K × noise spread (raise = stricter); lowered 25% from 4.5 for easier clench detection
JAW_SPREAD_MIN = 0.08   # spread floor as a fraction of baseline (keeps detection relative)
JAW_HIST_LEN   = 240    # samples of resting history per channel (~ a few seconds)
JAW_COOLDOWN   = 0.8    # s  minimum gap between counted jaw triggers

# ── Movement-start cooldowns ──────────────────────────────────────────────────
# When a FORWARD or BACKWARD movement begins, briefly reject new commands so
# blink/jaw noise can't bounce the drive state right after a move starts.
# Head-tilt steering is never affected (it only refines an already-active move).
FLIP_COOLDOWN = 0.20   # s  block the opposite direction (fwd↔back) this long
STOP_COOLDOWN = 0.06   # s  block STOP this long (kept short so a halt stays snappy)

# ── EEG-stall watchdog (fable fix 4) ───────────────────────────────────────────
# If the Muse BLE link stays up but no EEG packet has arrived for this long, the
# car is auto-STOPped until data resumes — no manual reset needed.
EEG_STALL_TIMEOUT = 1.0   # s

# ── Head-nod LOCK / head-hold UNLOCK (safety park) ─────────────────────────────
# A deliberate forward→back→forward→back head NOD sequence LOCKS the car: it holds
# STOP and ignores every drive command (blink / jaw / tilt) until it is unlocked.
# UNLOCK is a separate, deliberately-hard-to-hit hold: tilt the head LEFT and hold
# 3 s, then tilt RIGHT and hold 3 s. The lock uses the pitch axis (nodding); the
# unlock uses the roll axis (the same ear-to-shoulder tilt as steering) but requires
# a long sustained hold so ordinary turning can never trip it.
PITCH_THRESHOLD  = 20.0   # deg from resting pitch for a nod to register (forward/back)
# Either alternation order counts (whichever way "forward" maps on the IMU) — it is
# always four strictly-alternating nods: F·B·F·B or B·F·B·F.
LOCK_PATTERNS    = {('F', 'B', 'F', 'B'), ('B', 'F', 'B', 'F')}
LOCK_SEQ_LEN     = 4
LOCK_NOD_WINDOW  = 2.0    # s max gap between consecutive nods (slower → sequence resets)
UNLOCK_HOLD_SEC  = 1.6    # s to hold each side (LEFT then RIGHT) to unlock
UNLOCK_TIMEOUT   = 12.0   # s of no progress → a half-finished unlock resets

# ── Drive state ───────────────────────────────────────────────────────────────
# 0 = STOP, 1 = FORWARD, 2 = BACKWARD
DRIVE_LABELS = ['STOP  ■', 'FORWARD ▲', 'BACKWARD ▼']
DRIVE_COLORS = ['\033[91m', '\033[92m', '\033[93m']


# ── Signal decoders ───────────────────────────────────────────────────────────
def decode_eeg_4ch(data: bytes) -> np.ndarray:
    """14-bit LSB-first packed → shape (4 samples, 4 channels) in µV.

    Vectorized with numpy (fable fix 8): unpack all 224 bits of the 28-byte
    payload at once with np.unpackbits (bitorder='little' reproduces the
    original per-byte LSB-first bit order exactly), group into 16 chunks of
    14 bits, and reduce each chunk to its integer value with a single weighted
    dot-product instead of a Python-level bit-shifting loop per sample."""
    buf   = np.frombuffer(data[:28], dtype=np.uint8)
    bits  = np.unpackbits(buf, bitorder='little')          # 224 bits, LSB-first per byte
    bits14 = bits[:16 * 14].reshape(16, 14)                 # 16 samples × 14 bits
    weights = (1 << np.arange(14, dtype=np.uint32))
    raw = bits14.astype(np.uint32) @ weights                # shape (16,)
    return raw.astype(np.float32).reshape(4, 4) * EEG_SCALE


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


# ── Brain-wave band power (relative, numpy FFT — same as muse_athena_waves.py) ──
def rel_band_powers(samples: np.ndarray) -> dict:
    """samples: 1-D array of one channel's last WAVE_WIN_SAMPLES µV values.
    Returns {band_name: relative power 0-1} = that band's share of total 1-44 Hz.

    Hann-windowed periodogram PSD; band power = sum(PSD)*df over the band, then
    divided by the total over WAVE_TOTAL_LO..WAVE_TOTAL_HI."""
    x = samples - np.mean(samples)                       # remove DC offset
    spec = np.fft.rfft(x * _WAVE_HANN)
    psd = (np.abs(spec) ** 2) / (WAVE_FS * _WAVE_WIN_NORM)
    psd[1:-1] *= 2.0                                     # one-sided (DC/Nyquist not doubled)
    df = _WAVE_FREQS[1] - _WAVE_FREQS[0]
    tmask = (_WAVE_FREQS >= WAVE_TOTAL_LO) & (_WAVE_FREQS < WAVE_TOTAL_HI)
    total = float(np.sum(psd[tmask]) * df)
    if total <= 0.0:
        return {name: 0.0 for name, _, _ in WAVE_BANDS}
    out = {}
    for name, lo, hi in WAVE_BANDS:
        mask = (_WAVE_FREQS >= lo) & (_WAVE_FREQS < hi)
        out[name] = float(np.sum(psd[mask]) * df) / total
    return out


# ── Blink detector (FORWARD/STOP — fixed-µV, from normal controller) ───────────
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
        self.saturated    = False

    def process(self, samples: np.ndarray) -> bool:
        val = float(np.max(np.abs(samples)))
        self.peak_display = val
        self.saturated = val >= SATURATION
        if self.baseline is None:
            self.baseline = val
            return False
        if self.saturated:
            # Railed electrode = bad contact. Don't count saturation noise as
            # blinks; keep tracking baseline so detection resumes on good contact.
            self.in_spike = False
            self.baseline = 0.96 * self.baseline + 0.04 * val
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
                now = time.monotonic()
                if self.peak > self.min_peak and (now - self.last_blink) > COOLDOWN:
                    self.count     += 1
                    self.last_blink = now
                    self.lit_until  = now + SHOW_MS / 1000.0
                    return True
        return False

    def is_lit(self) -> bool:
        return time.monotonic() < self.lit_until


# ── Jaw clench detector (BACKWARD — drift-proof per-channel TP9/TP10 EMG) ───────
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
    trigger, so it tracks resting drift and rejects blinks (low-freq, on AF7/AF8).

    BACKWARD path lifted verbatim from muse_athena_controller_Adapt.py: only TP9
    needs to cross its trigger to count as a clench (TP10 is still tracked for the
    meter/baseline but no longer required), and there is no saturation guard."""

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

        now = time.monotonic()
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
        return time.monotonic() < self.lit_until


# ── Shared state ──────────────────────────────────────────────────────────────
# Fix 7: a small, consistent set of locks, one per logical state group. None of
# them are ever held across blocking I/O, so none add meaningful latency to the
# 20 Hz control loop or the BLE callback path.
#   _lock       — detection state (blink/jaw detectors, wave gate, blink-burst /
#                 mutual-priority state), IMU roll/pitch + lock/unlock state,
#                 drive state + cooldowns, sync timing, EEG-stall watchdog.
#   _conn_lock  — Muse / HC-08 connection status flags + status text.
#   _hc08_lock  — HC-08 command dedupe state (_last_cmd / _current_drive_cmd).
_lock      = threading.RLock()
_conn_lock = threading.Lock()

left_det   = BlinkDetector(LEFT_RISE_THRESH, LEFT_MIN_PEAK)   # AF7 — lower threshold
right_det  = BlinkDetector()                                  # AF8 — default threshold
jaw_det    = JawDetector()
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]
_pkt_count = 0
running    = True

# Last EEG packet arrival time (fable fix 4: stall watchdog heartbeat).
_last_eeg_pkt_t = 0.0

# Rolling per-channel EEG buffers for brain-wave band power (TP9, AF7, AF8, TP10).
# Fable fix 8: preallocated numpy ring buffer updated in place, instead of a
# deque that had to be rebuilt into a fresh np.array on every incoming packet.
_wave_ring      = np.zeros((4, WAVE_WIN_SAMPLES), dtype=np.float64)
_wave_write_idx = 0     # next write position, shared across all 4 channels
_wave_filled    = 0     # samples written so far, capped at WAVE_WIN_SAMPLES


def _wave_push(arr: np.ndarray) -> None:
    """Write one packet's samples (shape (n_samples, 4 channels)) into the
    preallocated per-channel ring buffers in place. Caller holds _lock."""
    global _wave_write_idx, _wave_filled
    n   = arr.shape[0]
    idx = _wave_write_idx
    end = idx + n
    if end <= WAVE_WIN_SAMPLES:
        _wave_ring[:, idx:end] = arr.T
    else:
        first = WAVE_WIN_SAMPLES - idx
        _wave_ring[:, idx:] = arr[:first].T
        _wave_ring[:, :n - first] = arr[first:].T
    _wave_write_idx = end % WAVE_WIN_SAMPLES
    _wave_filled = min(_wave_filled + n, WAVE_WIN_SAMPLES)


def _wave_window(ch: int) -> np.ndarray:
    """Chronological (oldest→newest) view of channel ch's rolling window, matching
    what np.array(deque) gave in the old deque-based buffer. Caller holds _lock."""
    if _wave_write_idx == 0:
        return _wave_ring[ch]
    return np.concatenate((_wave_ring[ch, _wave_write_idx:], _wave_ring[ch, :_wave_write_idx]))


# Short rolling history of gate band powers: (t, delta, beta, gamma) for the last
# WAVE_GATE_WINDOW seconds, so a detection can check whether a limit was reached at
# any tick in that window (not only the firing tick).
_gate_hist = deque()

# Last gate readings + reject counters (for the dashboard).
_blink_delta_rel = 0.0   # last Delta% measured at a blink attempt (max AF7/AF8)
_jaw_beta_rel    = 0.0    # last Beta% measured at a clench attempt (max TP9/TP10)
_jaw_gamma_rel   = 0.0    # last Gamma% measured at a clench attempt (max TP9/TP10)
_blink_gate_rej  = 0     # blinks rejected because Delta < min
_jaw_gate_rej    = 0     # clenches rejected because Beta/Gamma both < min

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

# Forward/back head-NOD state (pitch axis) for the LOCK gesture.
_imu_pitch_raw     = 0.0    # raw pitch from the accelerometer (deg)
_imu_pitch         = 0.0    # pitch after subtracting the resting offset
_imu_pitch_samples = []     # pitch readings gathered during sync → resting offset
_pitch_offset      = 0.0
_imu_pitch_cal     = False
_nod_dirs          = deque(maxlen=LOCK_SEQ_LEN)   # recent nod directions ('F'/'B')
_nod_zone          = 0     # current pitch zone: 0 neutral, +1 forward, -1 back
_nod_last_t        = 0.0   # time of the most recent registered nod

# Car LOCK state + head-hold UNLOCK progress (roll axis).
_locked             = False
_unlock_phase       = 0    # 0 = waiting LEFT-hold · 1 = LEFT done, waiting RIGHT-hold
_unlock_left_since  = 0.0  # start of the current continuous LEFT hold (0 = not held)
_unlock_right_since = 0.0  # start of the current continuous RIGHT hold (0 = not held)
_unlock_progress_t  = 0.0  # last time the unlock made progress (drives the timeout reset)

# Blink-burst resolution: 1 blink → FORWARD, 2+ blinks → STOP.
_blink_last_event  = 0.0   # last merged blink (de-dupes L+R of one physical blink)
_blink_burst_count = 0     # blinks counted in the current burst
_blink_burst_last  = 0.0   # time of the most recent blink in the burst
_blink_events      = 0     # debug: total blink events that passed the merge de-dupe
_last_blink_action = '—'   # debug: last resolved burst result (FWD / STOP / ignore1)
_blink_burst_gated = False # True once ANY blink in the current burst cleared the Delta
                           # gate. Lets a double-blink's 2nd blink in (gate passes once).

# Blink ↔ clench mutual-priority windows (MIX-only).
#   _jaw_blink_suppress_until : while now < this, a clench is active → blinks lose FORWARD.
#   _blink_jaw_suppress_until : while now < this, a blink is active → clenches may lose BACKWARD.
#   _blink_fwd_suppressed     : current blink burst is FORWARD-blocked (still counts toward STOP).
_jaw_blink_suppress_until = 0.0
_blink_jaw_suppress_until = 0.0
_blink_fwd_suppressed     = False

# Jaw clench resolution: a clench commits to BACKWARD immediately under the
# mutual-priority rule (see _register_jaw); _jaw_pending hands that off to the
# control loop, which consumes it once per tick.
_jaw_pending = False

# Connection status (shown on dashboard) ──────────────────────────────────────
_muse_connected = False
_muse_status    = 'Waiting...'
_hc08_connected = False
_hc08_status    = 'Waiting...'


def _set_muse_status(connected, status) -> None:
    """Update Muse connection dashboard fields under _conn_lock. Pass None for
    a field to leave it unchanged."""
    global _muse_connected, _muse_status
    with _conn_lock:
        if connected is not None:
            _muse_connected = connected
        if status is not None:
            _muse_status = status


def _set_hc08_status(connected, status) -> None:
    """Update HC-08 connection dashboard fields under _conn_lock. Pass None for
    a field to leave it unchanged."""
    global _hc08_connected, _hc08_status
    with _conn_lock:
        if connected is not None:
            _hc08_connected = connected
        if status is not None:
            _hc08_status = status


# _sync_start = inf means sync hasn't started yet → _is_syncing() stays True until
# it's set to time.monotonic() once the Muse has connected and started streaming.
_sync_start: float = float('inf')


# ── Brain-wave gates (windowed — limit may be reached at any tick in 30 ms) ────
def _update_wave_gate_hist() -> None:
    """Compute the gate-relevant relative band powers from the current 1 s window
    and push them onto _gate_hist (kept to the last WAVE_GATE_WINDOW seconds).
    Called once per EEG packet, BEFORE the detectors run (caller holds _lock).
    The dashboard readouts (_blink_delta_rel / _jaw_beta_rel / _jaw_gamma_rel) are
    refreshed as the MAX over that window, so a value that crossed its limit
    anywhere in the window is what the gate (and the display) sees — not just
    the instantaneous tick."""
    global _blink_delta_rel, _jaw_beta_rel, _jaw_gamma_rel
    if _wave_filled < WAVE_WIN_SAMPLES:
        return
    p_af7  = rel_band_powers(_wave_window(1))   # left eye
    p_af8  = rel_band_powers(_wave_window(2))   # right eye
    p_tp9  = rel_band_powers(_wave_window(0))   # left temple
    p_tp10 = rel_band_powers(_wave_window(3))   # right temple
    delta = max(p_af7['Delta'], p_af8['Delta'])        # blink → eye channels
    beta  = max(p_tp9['Beta'],  p_tp10['Beta'])        # clench → temple channels
    gamma = max(p_tp9['Gamma'], p_tp10['Gamma'])
    t = time.monotonic()
    _gate_hist.append((t, delta, beta, gamma))
    cutoff = t - WAVE_GATE_WINDOW
    while _gate_hist and _gate_hist[0][0] < cutoff:
        _gate_hist.popleft()
    # Windowed maxima drive both the gate decisions and the dashboard bars.
    _blink_delta_rel = max(d for (_t, d, _b, _g) in _gate_hist)
    _jaw_beta_rel    = max(b for (_t, _d, b, _g) in _gate_hist)
    _jaw_gamma_rel   = max(g for (_t, _d, _b, g) in _gate_hist)


def _blink_wave_gate() -> bool:
    """Blink confirmed if Delta reached DELTA_BLINK_MIN at ANY tick within the last
    WAVE_GATE_WINDOW (eye channels AF7/AF8 — _blink_delta_rel is that windowed max).
    Passes through until a full 1 s window has been buffered (sync covers startup)."""
    if not _gate_hist:
        return True
    return _blink_delta_rel >= DELTA_BLINK_MIN


def _jaw_wave_gate() -> bool:
    """Clench confirmed if Beta ≥ BETA_JAW_MIN OR Gamma ≥ GAMMA_JAW_MIN at ANY tick
    within the last WAVE_GATE_WINDOW (temple channels TP9/TP10). Either band is
    enough; if neither reaches its minimum the clench is rejected. Passes through
    until a full 1 s window has been buffered."""
    if not _gate_hist:
        return True
    return (_jaw_beta_rel >= BETA_JAW_MIN) or (_jaw_gamma_rel >= GAMMA_JAW_MIN)


# ── HC-08 BLE output thread ───────────────────────────────────────────────────
_hc08_queue = queue.SimpleQueue()
_hc08_lock  = threading.Lock()          # guards _last_cmd / _current_drive_cmd
_last_cmd: "str | None" = None
_current_drive_cmd: "str | None" = None   # latest command the control loop wants,
                                           # regardless of dedupe (fix 2: lets a
                                           # reconnect re-arm the real current command)
_hc08_loop: "asyncio.AbstractEventLoop | None" = None   # set once _hc08_ble_main starts
_hc08_cmd_event: "asyncio.Event | None" = None          # signalled on every send_cmd


def send_cmd(cmd: str) -> None:
    """Queue cmd for the HC-08 send loop, de-duped against the last cmd actually
    queued. _current_drive_cmd always tracks the latest desired command — even
    when deduped — so a BLE reconnect can force a re-send of it (fix 2)."""
    global _last_cmd, _current_drive_cmd
    with _hc08_lock:
        _current_drive_cmd = cmd
        if cmd == _last_cmd:
            return
        _last_cmd = cmd
    _hc08_queue.put(cmd)
    # Fix 8: wake the send loop immediately instead of it discovering the new
    # command on its next poll.
    if _hc08_loop is not None and _hc08_cmd_event is not None:
        _hc08_loop.call_soon_threadsafe(_hc08_cmd_event.set)


async def _hc08_ble_main() -> None:
    global _hc08_loop, _hc08_cmd_event, _last_cmd
    _hc08_loop = asyncio.get_running_loop()
    _hc08_cmd_event = asyncio.Event()
    while running:
        _set_hc08_status(False, 'Connecting...')
        try:
            async with BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                _set_hc08_status(True, f'Connected  {HC08_ADDRESS}')
                # Flush any stale commands queued while we were disconnected.
                while True:
                    try:    _hc08_queue.get_nowait()
                    except queue.Empty: break
                # Fix 2: a reconnect must not leave the dedupe state pointing at
                # whatever was last sent before the drop — that would silently
                # swallow the next send_cmd() call if the control loop's desired
                # command hasn't changed, leaving the fresh connection with only
                # the initial S+speed below and never the real drive command.
                # Reset the dedupe state and capture the current desired command
                # so it gets pushed out on this connection too.
                with _hc08_lock:
                    _last_cmd = None
                    cmd_to_resend = _current_drive_cmd
                # Send initial stop, then lock in the same speed as the keyboard tester
                await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
                await client.write_gatt_char(
                    UART_CHAR_UUID, DEFAULT_SPEED.encode(), response=True)
                if cmd_to_resend is not None:
                    send_cmd(cmd_to_resend)
                while client.is_connected and running:
                    drained = False
                    while True:
                        try:
                            raw = _hc08_queue.get_nowait()
                        except queue.Empty:
                            break
                        await client.write_gatt_char(
                            UART_CHAR_UUID, raw.encode(), response=True)
                        drained = True
                    if drained:
                        continue
                    # Fix 8: wait on an event instead of busy-polling at 50 Hz.
                    # send_cmd() wakes this immediately; the timeout is only a
                    # safety net so `running` / is_connected keep getting rechecked.
                    _hc08_cmd_event.clear()
                    try:
                        await asyncio.wait_for(_hc08_cmd_event.wait(), timeout=0.5)
                    except asyncio.TimeoutError:
                        pass
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
            _set_hc08_status(False, f'Not found — retrying ({type(e).__name__})')
        _set_hc08_status(False, None)
        if running:
            await asyncio.sleep(2.0)


def _hc08_thread() -> None:
    asyncio.run(_hc08_ble_main())


# ── Sync helpers ──────────────────────────────────────────────────────────────
def _is_syncing() -> bool:
    with _lock:
        start = _sync_start
    return (time.monotonic() - start) < SYNC_DURATION


def _sync_remaining() -> float:
    with _lock:
        start = _sync_start
    return max(0.0, SYNC_DURATION - (time.monotonic() - start))


# ── Blink-burst resolver ──────────────────────────────────────────────────────
def _register_blink(now: float) -> None:
    """Count blinks into a burst (caller holds _lock). L+R of one physical blink
    (within BLINK_MERGE) collapse to one. A clench active in the priority window
    can suppress this blink's FORWARD (see the mutual-priority comment block)."""
    global _blink_last_event, _blink_burst_count, _blink_burst_last
    global _blink_events, _blink_fwd_suppressed
    global _blink_jaw_suppress_until
    if now - _blink_last_event < BLINK_MERGE:
        return   # same physical blink firing twice (e.g. AF7 then AF8)
    _blink_last_event = now
    _blink_events    += 1                 # debug counter
    if _blink_burst_count > 0 and (now - _blink_burst_last) <= MULTI_WINDOW:
        _blink_burst_count += 1          # another blink in the same burst
    else:
        _blink_burst_count    = 1        # new burst
        _blink_fwd_suppressed = False    # fresh burst starts un-blocked
    _blink_burst_last = now
    if now < _jaw_blink_suppress_until:
        # A clench is active → it came first (or won a tie) → this blink loses its
        # FORWARD. It still counts toward a double-blink STOP. The flag is cleared
        # when the burst resolves / the window closes (see _resolve_blinks).
        _blink_fwd_suppressed = True
    else:
        # FORWARD wins for now → open the forward window so a clench detected later
        # (beyond the 200 ms backward tie-reach) is treated as bleed and loses its
        # BACKWARD. A FORWARD-blocked blink does not open this window.
        _blink_jaw_suppress_until = now + BLINK_JAW_PRIORITY_POST


def _resolve_blinks(now: float):
    """Run in control loop (caller holds _lock). 1 blink → FORWARD, 2+ → STOP.
    STOP fires the moment a 2nd blink lands (instant, no wait). A lone blink
    fires FORWARD only once the MULTI_WINDOW closes with no 2nd blink.
    Returns (forward, stop)."""
    global _blink_burst_count, _last_blink_action, _blink_fwd_suppressed
    global _blink_burst_gated
    forward = stop = False
    if _blink_burst_count >= 2:
        # Double-blink STOP always wins — honoured even inside a clench window.
        stop = True
        _last_blink_action = f'STOP({_blink_burst_count})'
        _blink_burst_count = 0
        _blink_fwd_suppressed = False
        _blink_burst_gated = False
    elif (_blink_fwd_suppressed and _blink_burst_count == 1
          and now >= _jaw_blink_suppress_until):
        # Clench-priority: discard the lone in-window blink the instant the
        # 400 ms window closes. It never fires FORWARD and is not cached, so a
        # FORWARD needs a fresh blink landing after the window.
        _blink_burst_count = 0
        _blink_fwd_suppressed = False
        _blink_burst_gated = False
        _last_blink_action = 'JAW>FWD'
    elif (_blink_burst_count == 1 and not _blink_fwd_suppressed
          and (now - _blink_burst_last) > MULTI_WINDOW):
        forward = True
        _last_blink_action = 'FWD(1)'
        _blink_burst_count = 0
        _blink_burst_gated = False
    return forward, stop


# ── Jaw-clench arming + clench-priority over blink (MIX-only) ──────────────────
def _register_jaw(now: float) -> None:
    """Resolve a detected clench under the mutual-priority rule (first-detected
    wins, ties → BACKWARD). Caller holds _lock. If an earlier FORWARD blink is
    clearly first — its forward window is still open AND the blink was more than
    the 200 ms tie-reach ago — the blink wins and this BACKWARD is dropped (not
    cached). Otherwise BACKWARD wins (clench first, independent, or a tie) and
    commits immediately; a coincident blink within the 200 ms pre-reach loses its
    FORWARD (still counts toward STOP). BACKWARD never waits for a window — the
    car reverses at once."""
    global _jaw_pending, _jaw_blink_suppress_until
    global _blink_fwd_suppressed, _last_blink_action, _blink_jaw_suppress_until

    if (now < _blink_jaw_suppress_until
            and (now - _blink_last_event) > JAW_BLINK_PRIORITY_PRE):
        # FORWARD was clearly first → it wins. BACKWARD is dropped and NOT cached,
        # so reversing needs a fresh clench after the forward window closes.
        _last_blink_action = 'FWD>JAW'
        return

    # BACKWARD wins. Commit immediately and open the backward window (blocks FORWARD).
    _jaw_pending = True
    _jaw_blink_suppress_until = now + JAW_BLINK_PRIORITY_POST
    _blink_jaw_suppress_until = 0.0    # clear any forward window — backward owns it now
    # A coincident blink (within the 200 ms tie-reach before the clench) loses its
    # FORWARD — it was bleed / a tie. A double-blink STOP (burst >= 2) is left alone.
    if (_blink_burst_count > 0 and _blink_burst_count < 2
            and (now - _blink_last_event) <= JAW_BLINK_PRIORITY_PRE):
        _blink_fwd_suppressed = True
        _last_blink_action    = 'JAW>FWD'


# ── Head-nod LOCK + head-hold UNLOCK gestures ─────────────────────────────────
def _feed_lock_gesture(pitch: float, now: float) -> None:
    """Watch the pitch (forward/back NOD) axis for the lock sequence (caller holds
    _lock). Entering the forward zone registers 'F', the back zone 'B'; four
    strictly-alternating nods (F·B·F·B or B·F·B·F) within LOCK_NOD_WINDOW of each
    other lock the car. Returning to neutral registers nothing — you must swing to
    the opposite extreme for the next nod, which is exactly the deliberate
    nod-nod-nod-nod motion asked for."""
    global _nod_zone, _nod_last_t, _locked, _unlock_phase
    global _unlock_left_since, _unlock_right_since, _unlock_progress_t
    if pitch > PITCH_THRESHOLD:
        zone = 1
    elif pitch < -PITCH_THRESHOLD:
        zone = -1
    else:
        zone = 0
    if zone == _nod_zone:
        return                        # no zone change → nothing new to register
    _nod_zone = zone
    if zone == 0:
        return                        # returned to neutral → not a nod on its own
    if _nod_dirs and now - _nod_last_t > LOCK_NOD_WINDOW:
        _nod_dirs.clear()             # too slow since the last nod → start over
    _nod_dirs.append('F' if zone == 1 else 'B')
    _nod_last_t = now
    if len(_nod_dirs) == LOCK_SEQ_LEN and tuple(_nod_dirs) in LOCK_PATTERNS:
        _locked             = True    # armed — car parks in STOP until unlocked
        _nod_dirs.clear()
        _unlock_phase       = 0       # fresh unlock progress
        _unlock_left_since  = 0.0
        _unlock_right_since = 0.0
        _unlock_progress_t  = now


def _feed_unlock_gesture(roll: float, now: float) -> None:
    """While locked, unlock on a sustained two-step hold: tilt LEFT and hold for
    UNLOCK_HOLD_SEC, then tilt RIGHT and hold for UNLOCK_HOLD_SEC (caller holds
    _lock). Each side must be held continuously — releasing before its time resets
    that side. No progress for UNLOCK_TIMEOUT rewinds a half-finished unlock back
    to the start."""
    global _locked, _unlock_phase, _unlock_left_since, _unlock_right_since
    global _unlock_progress_t
    left  = roll < -ROLL_THRESHOLD
    right = roll >  ROLL_THRESHOLD
    if _unlock_phase == 0:                          # phase 0: hold LEFT
        if left:
            if _unlock_left_since == 0.0:
                _unlock_left_since = now
            _unlock_progress_t = now
            if now - _unlock_left_since >= UNLOCK_HOLD_SEC:
                _unlock_phase       = 1             # LEFT satisfied → now hold RIGHT
                _unlock_right_since = 0.0
        else:
            _unlock_left_since = 0.0                # must hold continuously
    else:                                           # phase 1: hold RIGHT
        if right:
            if _unlock_right_since == 0.0:
                _unlock_right_since = now
            _unlock_progress_t = now
            if now - _unlock_right_since >= UNLOCK_HOLD_SEC:
                _locked             = False         # unlocked — drive resumes
                _unlock_phase       = 0
                _unlock_left_since  = 0.0
                _unlock_right_since = 0.0
        else:
            _unlock_right_since = 0.0
    if _unlock_phase != 0 and now - _unlock_progress_t > UNLOCK_TIMEOUT:
        _unlock_phase       = 0                     # stalled half-unlock → reset
        _unlock_left_since  = 0.0
        _unlock_right_since = 0.0


# ── Sensor callback ───────────────────────────────────────────────────────────
def on_sensor(handle, data: bytearray):
    global _pkt_count
    global _imu_roll_raw, _imu_roll, _imu_roll_samples
    global _roll_offset, _imu_calibrated, _imu_ts
    global _imu_pitch_raw, _imu_pitch, _imu_pitch_samples, _pitch_offset, _imu_pitch_cal
    global _blink_gate_rej, _jaw_gate_rej, _blink_burst_gated, _last_eeg_pkt_t

    _pkt_count += 1
    subpackets = parse_payload(bytes(data))
    for tag, stype, raw in subpackets:

        if tag == 0x11:   # EEG 4-channel
            arr = decode_eeg_4ch(raw)
            now = time.monotonic()
            with _lock:
                _last_eeg_pkt_t = now   # fix 4: EEG-stall watchdog heartbeat

                # Feed the rolling band-power ring buffers (TP9, AF7, AF8, TP10)
                # BEFORE the gates run, so the window includes the spike/EMG that
                # just fired.
                _wave_push(arr)
                _update_wave_gate_hist()

                blink_l = left_det.process(arr[:, 1])    # AF7 — left eye
                blink_r = right_det.process(arr[:, 2])   # AF8 — right eye
                jaw_fired = jaw_det.process(arr, _is_syncing())   # TP9/TP10 clench

                for ch in range(4):
                    _eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))

                # Evaluate both detections + their wave gates up front. The gates
                # read the 30 ms windowed-max band powers (see _update_wave_gate_hist).
                active     = not _is_syncing()
                blink_now  = (blink_l or blink_r) and active
                jaw_now    = jaw_fired and active
                blink_gate = _blink_wave_gate() if blink_now else False
                jaw_gate   = _jaw_wave_gate()   if jaw_now   else False

                # PRIORITY: a blink AND a clench in the SAME packet, with BOTH wave
                # minimums met (Delta for the blink AND Beta/Gamma for the clench),
                # resolves to BACKWARD — the clench wins the tie and the blink is
                # dropped. Exception: if the blink completes a double-blink, STOP
                # wins (safety) even alongside a clench.
                if blink_now and jaw_now and blink_gate and jaw_gate:
                    completing_double = (_blink_burst_count >= 1
                                         and (now - _blink_burst_last) <= MULTI_WINDOW)
                    if completing_double:
                        _register_blink(now)        # 2nd blink → STOP wins
                        _blink_burst_gated = True
                    else:
                        _register_jaw(now)          # single blink + clench → BACKWARD
                else:
                    # Independent handling.
                    # BLINK accepted if its Delta gate passes, or if it continues an
                    # already-gated burst (a double-blink needs Delta to pass only once).
                    if blink_now:
                        burst_cont = (_blink_burst_count > 0
                                      and (now - _blink_burst_last) <= MULTI_WINDOW
                                      and _blink_burst_gated)
                        if blink_gate or burst_cont:
                            _register_blink(now)
                            if blink_gate:
                                _blink_burst_gated = True
                        else:
                            _blink_gate_rej += 1
                    # CLENCH arms BACKWARD immediately (clench-priority window — see
                    # _register_jaw), accepted only if Beta ≥ 15% OR Gamma ≥ 20%.
                    if jaw_now:
                        if jaw_gate:
                            _register_jaw(now)
                        else:
                            _jaw_gate_rej += 1

        elif tag == 0x47:   # ACCGYRO → head-tilt (roll) steering + nod (pitch) lock
            imu = decode_accgyro(raw)
            ax, ay, az = float(imu[-1, 0]), float(imu[-1, 1]), float(imu[-1, 2])
            now_i = time.monotonic()
            with _lock:
                _imu_roll_raw  = math.degrees(math.atan2(ay, az))
                _imu_pitch_raw = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                _imu_ts = now_i
                if _is_syncing():
                    _imu_roll_samples.append(_imu_roll_raw)    # gather resting offsets
                    _imu_pitch_samples.append(_imu_pitch_raw)
                else:
                    if not _imu_calibrated and _imu_roll_samples:
                        _roll_offset    = float(np.mean(_imu_roll_samples))
                        _imu_calibrated = True
                    if not _imu_pitch_cal and _imu_pitch_samples:
                        _pitch_offset  = float(np.mean(_imu_pitch_samples))
                        _imu_pitch_cal = True
                    _imu_roll  = _imu_roll_raw  - _roll_offset
                    _imu_pitch = _imu_pitch_raw - _pitch_offset
                    # LOCK/UNLOCK gestures: while locked, watch roll for the unlock
                    # hold; otherwise watch pitch for the forward/back nod lock sequence.
                    if _locked:
                        _feed_unlock_gesture(_imu_roll, now_i)
                    else:
                        _feed_lock_gesture(_imu_pitch, now_i)


def on_ctrl(handle, data: bytearray):
    pass


# ── Control logic (called at 20 Hz from UI thread) ────────────────────────────
def update_control() -> None:
    global _drive_state, _jaw_pending
    global _flip_cooldown_until, _stop_cooldown_until

    now = time.monotonic()   # fix 10: one tick timestamp, reused for the whole tick

    with _lock:
        forward, stop = _resolve_blinks(now)   # 1 blink → FORWARD · 2+ blinks → STOP

        # A detected clench commits to BACKWARD immediately. The JawDetector already
        # gates on TP9 over its adaptive trigger plus JAW_COOLDOWN, so a clench is
        # trustworthy on its own — no blink-bleed confirm delay needed.
        jaw_back = False
        if _jaw_pending:
            _jaw_pending = False
            jaw_back = True

        # LOCK: a forward→back→forward→back head nod parks the car. While locked it
        # holds STOP and ignores every drive command (blink/jaw/tilt). The pending
        # jaw was already consumed above so it can't fire the instant we unlock.
        # Unlock is the hold-LEFT-then-RIGHT head gesture, handled in the IMU callback.
        if _locked:
            _drive_state = 0
            send_cmd('S')
            return

        # Movement-start cooldowns: while a fresh move is settling, drop the commands
        # that would fight it. FLIP_COOLDOWN blocks the opposite direction (so noise
        # can't flip fwd↔back); STOP_COOLDOWN briefly blocks STOP. Head-tilt steering
        # below is untouched. STOP after its short window still wins for safety.
        if now < _flip_cooldown_until:
            forward  = False
            jaw_back = False
        if now < _stop_cooldown_until:
            stop = False

        # DIRECTION LOCK (stop-switch build): never flip directly between FORWARD and
        # BACKWARD — a STOP must come in between. While going FORWARD, ignore a reverse
        # command; while going BACKWARD, ignore a forward command. STOP is unaffected,
        # so a double-blink is the only way to switch direction. Gated on the current
        # (pre-tick) drive state.
        if jaw_back and _drive_state == 1:   # already FORWARD → can't reverse without a STOP
            jaw_back = False
        if forward and _drive_state == 2:    # already BACKWARD → can't go forward without a STOP
            forward = False

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

        with _conn_lock:
            muse_connected = _muse_connected

        # Fix 4: EEG-stall watchdog — if the Muse link is up but no EEG packet has
        # arrived for over EEG_STALL_TIMEOUT, auto-STOP; resumes automatically the
        # moment packets start flowing again (drive_state itself is left alone, so
        # whatever was latched just resumes once the stall clears — same pattern as
        # the sync gate below).
        stalled = (_last_eeg_pkt_t > 0.0) and ((now - _last_eeg_pkt_t) > EEG_STALL_TIMEOUT)

        # Safety: never drive unless the Muse is connected, the sync is done, and
        # EEG isn't stalled.
        if not muse_connected or _is_syncing() or stalled:
            send_cmd('S')
            return

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


def _gate_bar(rel: float, thresh: float, width: int = 12) -> str:
    """Relative-power bar (0-1 full scale) that turns green once it clears thresh."""
    frac   = max(0.0, min(rel, 1.0))
    filled = int(frac * width)
    col    = '\033[92m' if rel >= thresh else '\033[91m'
    return col + '█' * filled + '\033[90m' + '░' * (width - filled) + '\033[0m'


def _wave_gate_str() -> str:
    """Brain-wave gate readout: last Delta% (blink) and Beta/Gamma% (jaw) measured,
    each against its minimum, plus how many detections the gate has rejected."""
    d_ok = _blink_delta_rel >= DELTA_BLINK_MIN
    d_mark = '\033[92m✓\033[0m' if d_ok else '\033[91m✗\033[0m'
    return (f'Blink Δ {_gate_bar(_blink_delta_rel, DELTA_BLINK_MIN)} '
            f'{_blink_delta_rel*100:4.0f}%/{DELTA_BLINK_MIN*100:.0f}% {d_mark} rej:{_blink_gate_rej}')


def _wave_gate_str2() -> str:
    j_ok = (_jaw_beta_rel >= BETA_JAW_MIN) or (_jaw_gamma_rel >= GAMMA_JAW_MIN)
    j_mark = '\033[92m✓\033[0m' if j_ok else '\033[91m✗\033[0m'
    return (f'Jaw   β {_gate_bar(_jaw_beta_rel, BETA_JAW_MIN)} '
            f'{_jaw_beta_rel*100:4.0f}%/{BETA_JAW_MIN*100:.0f}%  '
            f'γ {_gate_bar(_jaw_gamma_rel, GAMMA_JAW_MIN)} '
            f'{_jaw_gamma_rel*100:4.0f}%/{GAMMA_JAW_MIN*100:.0f}% {j_mark} rej:{_jaw_gate_rej}')


def _roll_str() -> str:
    """Head-tilt (roll) steering indicator for the dashboard."""
    has_imu = _imu_ts > 0 and (time.monotonic() - _imu_ts) <= 2.0
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


def _lock_str() -> str:
    """LOCK/UNLOCK line for the dashboard: when locked, show which unlock hold is
    pending and its progress; when unlocked, show the nod-lock hint + partial seq."""
    if _locked:
        if _unlock_phase == 0:
            held = (time.monotonic() - _unlock_left_since) if _unlock_left_since else 0.0
            step = (f'hold \033[1;96mLEFT\033[0m {min(held, UNLOCK_HOLD_SEC):.1f}/'
                    f'{UNLOCK_HOLD_SEC:.0f}s, then RIGHT')
        else:
            held = (time.monotonic() - _unlock_right_since) if _unlock_right_since else 0.0
            step = (f'\033[92mLEFT ✓\033[0m · hold \033[1;96mRIGHT\033[0m '
                    f'{min(held, UNLOCK_HOLD_SEC):.1f}/{UNLOCK_HOLD_SEC:.0f}s')
        return f'\033[1;97;41m 🔒 LOCKED \033[0m  unlock → {step}'
    seq = ''.join(_nod_dirs) if _nod_dirs else '—'
    return (f'\033[92m 🔓 unlocked \033[0m  \033[90mnod F·B·F·B to lock   '
            f'seq:{seq}\033[0m')


def _jaw_str() -> str:
    """Jaw-clench meter: per-channel fraction toward the adaptive trigger.
    (BACKWARD display from the Adapt controller — no NO-CONTACT tag.)"""
    j   = jaw_det
    f9  = (j.emg9  - j.emg9_base)  / (j.emg9_trig  - j.emg9_base)  if j.emg9_trig  > j.emg9_base  else 0.0
    f10 = (j.emg10 - j.emg10_base) / (j.emg10_trig - j.emg10_base) if j.emg10_trig > j.emg10_base else 0.0
    tag = '\033[1;97;41m JAW! \033[0m' if j.is_lit() else '\033[90m ---- \033[0m'
    return f'TP9 {_frac_bar(f9)}  TP10 {_frac_bar(f10)}  {tag} cnt:{j.count}'


def draw():
    global _first_frame

    with _conn_lock:
        muse_connected, muse_status = _muse_connected, _muse_status
        hc08_connected, hc08_status = _hc08_connected, _hc08_status
    with _hc08_lock:
        last_cmd = _last_cmd

    # Fix 7: one lock acquisition snapshots + formats everything else, so the
    # frame can't be built from a mix of before/after values torn across threads.
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
        if not muse_connected:
            banner = '  \033[91m── WAITING FOR MUSE HEADSET ───────────────────────────────\033[0m'
        elif syncing:
            banner = (f'  \033[93m── SYNCING  {rem:.0f}s remaining  '
                      f'(keep headset still, let EEG settle) ──\033[0m')
        else:
            banner = '  \033[92m── ACTIVE ─────────────────────────────────────────────────\033[0m'

        # Drive state line
        if not muse_connected or syncing:
            drive_str = (f'  Drive:  {DRIVE_COLORS[ds]}{DRIVE_LABELS[ds]}\033[0m'
                         f'   \033[90m(not active yet)\033[0m')
        else:
            drive_str = (f'  Drive:  {DRIVE_COLORS[ds]}{DRIVE_LABELS[ds]}\033[0m'
                         f'   \033[90mblink → FWD · jaw → BACK · 2× blink → STOP\033[0m')

        rows = [
            f'  \033[1mMuse S Athena\033[0m — Car Controller (ALPHA · wave-gated · stop-switch)   [{time.strftime("%H:%M:%S")}]',
            f'  Muse   {_conn_cell(muse_connected, muse_status)}   \033[90mpkts:{_pkt_count}\033[0m',
            f'  HC-08  {_conn_cell(hc08_connected, hc08_status)}',
            f'',
            banner,
            f'',
            drive_str,
            f'  Steer:  {_roll_str()}',
            f'  Lock:   {_lock_str()}',
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
            f'  \033[96m── WAVE GATE ───────────────────────────────────────────────\033[0m',
            f'  {_wave_gate_str()}',
            f'  {_wave_gate_str2()}',
            f'',
            f'  CMD: {_CMD_DESC.get(last_cmd, "---")}   '
            f'\033[90mblinks: burst={_blink_burst_count} events={_blink_events} last={_last_blink_action}\033[0m',
            f'  \033[90mCtrl-C quit · r=recalibrate · blink=FWD · 2 blinks=STOP · jaw=BACK · head-tilt=turn\033[0m',
            f'  \033[90mnod F·B·F·B = LOCK · hold tilt LEFT 3s then RIGHT 3s = UNLOCK\033[0m',
        ]

    # Flicker-free repaint: move the cursor back up to the top of the block and
    # overwrite each line in place, clearing only that line's tail with \033[K.
    # Fix 1: the cursor-reposition amount is derived from len(rows) — the actual
    # number of rows just emitted — instead of a separate hardcoded constant that
    # could (and did) drift out of sync with them and scroll the dashboard.
    n = len(rows)
    if _first_frame:
        prefix = '\n' * n     # first frame: just reserve space, never move up
        _first_frame = False
    else:
        prefix = f'\033[{n}A'
    frame = prefix + ''.join(row + '\033[K\n' for row in rows)
    sys.stdout.write(frame)
    sys.stdout.flush()


def _reset_calibration() -> None:
    """'r' key handler: restart the 30 s calibration live so the user can re-tune
    on the fly. Re-zeros head-tilt roll, rebuilds blink + jaw baselines, clears
    blink counts/bursts, and parks the car in STOP for safety."""
    global left_det, right_det, jaw_det, _sync_start
    global _imu_calibrated, _imu_roll_samples, _roll_offset, _imu_roll
    global _blink_burst_count, _blink_events, _last_blink_action, _drive_state
    global _flip_cooldown_until, _stop_cooldown_until
    global _jaw_blink_suppress_until, _blink_jaw_suppress_until, _blink_fwd_suppressed
    global _blink_delta_rel, _jaw_beta_rel, _jaw_gamma_rel, _blink_gate_rej, _jaw_gate_rej
    global _blink_burst_gated
    global _imu_pitch_cal, _imu_pitch_samples, _pitch_offset, _imu_pitch, _nod_zone
    global _locked, _unlock_phase, _unlock_left_since, _unlock_right_since, _unlock_progress_t
    global _wave_write_idx, _wave_filled, _last_eeg_pkt_t
    with _lock:
        left_det  = BlinkDetector(LEFT_RISE_THRESH, LEFT_MIN_PEAK)
        right_det = BlinkDetector()
        jaw_det   = JawDetector()
        _wave_ring[:]   = 0.0
        _wave_write_idx = 0
        _wave_filled    = 0
        _gate_hist.clear()
        _imu_calibrated    = False
        _imu_roll_samples  = []
        _roll_offset       = 0.0
        _imu_roll          = 0.0
        _imu_pitch_cal     = False
        _imu_pitch_samples = []
        _pitch_offset      = 0.0
        _imu_pitch         = 0.0
        _nod_zone          = 0
        _nod_dirs.clear()
        _locked            = False
        _unlock_phase      = 0
        _unlock_left_since = 0.0
        _unlock_right_since= 0.0
        _unlock_progress_t = 0.0
        _blink_burst_count = 0
        _blink_burst_gated = False
        _blink_events      = 0
        _last_blink_action = '—'
        _drive_state       = 0
        _flip_cooldown_until = 0.0
        _stop_cooldown_until = 0.0
        _jaw_blink_suppress_until = 0.0
        _blink_jaw_suppress_until = 0.0
        _blink_fwd_suppressed     = False
        _blink_delta_rel   = 0.0
        _jaw_beta_rel      = 0.0
        _jaw_gamma_rel     = 0.0
        _blink_gate_rej    = 0
        _jaw_gate_rej      = 0
        _last_eeg_pkt_t    = 0.0
        _sync_start        = time.monotonic()   # restart the 30 s sync
    send_cmd('S')


def _ui_thread() -> None:
    """Dashboard + control loop. Runs the whole program lifetime so connection
    status is visible even while the Muse/HC-08 are still connecting."""
    ctrl_tick = draw_tick = time.monotonic()
    while running:
        now = time.monotonic()
        # 'r' restarts calibration live so the user can re-tune on the fly.
        # Fix 9: drain ALL pending keypresses this tick, not just the first one.
        if msvcrt is not None:
            while msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ('r', 'R'):
                    _reset_calibration()
        # Control stays at 20 Hz for responsive commands; the dashboard redraws at
        # ~10 Hz so the console isn't flooded (the old 20 Hz full redraw was the lag).
        if now - ctrl_tick >= 0.05:
            update_control()
            ctrl_tick = now
        if now - draw_tick >= 0.10:
            draw()
            draw_tick = now
        time.sleep(0.02)


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
    global running, _sync_start, _drive_state

    while running:
        _set_muse_status(False, None)
        try:
            _set_muse_status(None, 'Scanning for any Muse headset...')
            _mlog('scanning...')
            # Scan for ALL nearby BLE devices and pick the first Muse, regardless
            # of MAC address — so any Muse headset works, not just one specific unit.
            found = await BleakScanner.discover(timeout=12.0)
            device = next(
                (d for d in found if d.name and 'muse' in d.name.lower()),
                None,
            )
            if device is None:
                _set_muse_status(None, 'No Muse found — is the headset on? Retrying...')
                _mlog(f'no muse; saw {[(d.address, d.name) for d in found]}')
                await asyncio.sleep(2.0)
                continue

            _set_muse_status(None, f'Found {device.name} ({device.address}) — connecting...')
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
                    _set_muse_status(None, 'GATT not ready — reconnecting...')
                    _mlog('GATT not ready (chars missing) — reconnecting')
                    await asyncio.sleep(1.0)
                    continue

                _set_muse_status(None, 'Connected — running init sequence...')
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
                with _lock:
                    if _sync_start == float('inf'):
                        _sync_start = time.monotonic()

                _set_muse_status(True, f'Streaming  {device.name} ({device.address})')
                _mlog('init complete — streaming')

                # Keep the connection alive until it drops or we quit
                while client.is_connected and running:
                    await asyncio.sleep(0.2)
                _mlog(f'connection loop exited (is_connected={client.is_connected})')

        except Exception as e:
            _mlog(f'EXCEPTION {type(e).__name__}: {e}')
            _set_muse_status(None, f'Lost / failed ({type(e).__name__}) — retrying...')

        # Fix 3: whatever caused us to fall out of the block above — a real
        # disconnect after streaming, or a failed/aborted connection attempt —
        # force the car to STOP now instead of letting it silently resume the
        # previous direction the instant the Muse reconnects.
        _set_muse_status(False, None)
        with _lock:
            _drive_state = 0
        send_cmd('S')
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
