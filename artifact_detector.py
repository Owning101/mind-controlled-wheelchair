"""
Muse 2 EEG artifact detector — blink, double-blink, and jaw-clench.

Usage:
    detector = ArtifactDetector(
        on_blink        = lambda: ...,   # single blink
        on_double_blink = lambda: ...,   # two blinks within 0.65 s
        on_jaw_clench   = lambda: ...,   # jaw clench
    )

    def eeg_callback(data, timestamps):
        detector.process(data)

Channel order expected: TP9=0, AF7=1, AF8=2, TP10=3
"""

import time
import numpy as np
from collections import deque

# ── Tuning ────────────────────────────────────────────────────────────────────
BLINK_THRESHOLD     = 150    # µV  peak deviation above quiet baseline on AF7/AF8
BLINK_COOLDOWN      = 0.60   # s   min gap between any blink detections
DOUBLE_BLINK_WINDOW = 0.65   # s   second blink within this window → double-blink
CLENCH_THRESHOLD    = 50     # µV  RMS required on BOTH TP9 and TP10
CLENCH_COOLDOWN     = 0.80   # s   min gap between clench events


class ArtifactDetector:
    """
    Detects single blinks, double blinks, and jaw clenches from Muse 2 EEG.

    Parameters
    ----------
    on_blink        : callable — fired on a single blink (default: count)
    on_double_blink : callable — fired when two blinks occur within DOUBLE_BLINK_WINDOW
    on_jaw_clench   : callable — fired on jaw clench (default: count)
    """

    def __init__(self, on_blink=None, on_double_blink=None, on_jaw_clench=None):
        self.on_blink        = on_blink        or self._count_blink
        self.on_double_blink = on_double_blink or self._count_double
        self.on_jaw_clench   = on_jaw_clench   or self._count_clench

        self.blink_count        = 0
        self.double_blink_count = 0
        self.clench_count       = 0
        self.blink_log          = deque(maxlen=6)
        self.clench_log         = deque(maxlen=6)

        self._baseline    = [None] * 4
        self._last_blink  = 0.0   # timestamp of most recent blink
        self._last_clench = 0.0
        self.baseline_ready = False

    # ── default no-callback actions ───────────────────────────────────────────
    def _count_blink(self):
        self.blink_count += 1
        self.blink_log.append(time.strftime('%H:%M:%S'))

    def _count_double(self):
        self.double_blink_count += 1

    def _count_clench(self):
        self.clench_count += 1
        self.clench_log.append(time.strftime('%H:%M:%S'))

    # ── main entry point ──────────────────────────────────────────────────────
    def process(self, data):
        """
        Feed a muselsl EEG packet into the detector.
        data : sequence of 4+ arrays, shape (n_channels, n_samples)
        """
        if len(data) < 4:
            return

        peaks = [float(np.max(np.abs(ch))) for ch in data[:4]]
        rms   = [float(np.sqrt(np.mean(np.square(ch)))) for ch in data[:4]]
        now   = time.time()

        # ── Baseline update (AF7=1, AF8=2) ───────────────────────────────────
        for ch in [1, 2]:
            val = float(np.mean(data[ch]))
            b   = self._baseline[ch]
            if b is None:
                self._baseline[ch] = val
            elif peaks[ch] < BLINK_THRESHOLD * 0.75:
                self._baseline[ch] = 0.97 * b + 0.03 * val

        self.baseline_ready = all(self._baseline[ch] is not None for ch in [1, 2])

        # ── Blink detection ───────────────────────────────────────────────────
        if self.baseline_ready and now - self._last_blink > BLINK_COOLDOWN:
            for ch in [1, 2]:
                if peaks[ch] - abs(self._baseline[ch]) > BLINK_THRESHOLD:
                    if self._last_blink > 0 and now - self._last_blink <= DOUBLE_BLINK_WINDOW:
                        # Second blink within window → double blink
                        self._last_blink = 0.0   # reset so the next blink starts fresh
                        self.on_double_blink()
                    else:
                        self._last_blink = now
                        self.on_blink()
                    break  # one blink event per EEG packet

        # ── Jaw clench (TP9=0, TP10=3) ───────────────────────────────────────
        # High-frequency EMG spikes both ear electrodes simultaneously.
        # Movement artifacts are typically asymmetric.
        if now - self._last_clench > CLENCH_COOLDOWN:
            if rms[0] > CLENCH_THRESHOLD and rms[3] > CLENCH_THRESHOLD:
                self._last_clench = now
                self.on_jaw_clench()
