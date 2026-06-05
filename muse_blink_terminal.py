#!/usr/bin/env python3
"""
muse_blink_terminal.py
Port of https://github.com/urish/muse-blink — terminal version.

Blink detection: spike pattern (rise above baseline → peak → fall back down).
NOT threshold-crossing — a sustained high value does NOT count.
"""

import sys
import time
import signal
from muselsl.stream import find_muse
from muselsl.muse import Muse

# ── Tuning ────────────────────────────────────────────────────────────────────
RISE_THRESH  = 120    # µV above baseline to start a spike
MIN_PEAK     = 180    # µV minimum absolute peak to qualify as blink
FALL_FRAC    = 0.40   # spike must fall to 40% of (peak-baseline) to confirm
COOLDOWN     = 0.35   # seconds minimum between two blinks on same eye
SHOW_MS      = 500    # ms to keep BLINK indicator lit after detection

# ── Spike state machine ───────────────────────────────────────────────────────
class BlinkDetector:
    """
    States:
      IDLE    → baseline tracking, waiting for rise
      SPIKING → saw big rise, tracking peak, waiting for fall
      → on confirmed fall: count blink, back to IDLE
    """
    def __init__(self):
        self.baseline    = None   # EMA of calm signal
        self.in_spike    = False
        self.peak        = 0.0
        self.count       = 0
        self.last_blink  = 0.0   # timestamp of last confirmed blink
        self.lit_until   = 0.0   # timestamp until indicator stays lit
        self.peak_display = 0.0  # current peak value (for the bar)

    def process(self, samples):
        """Call with one chunk of samples from AF7 or AF8. Returns True on blink."""
        val = max(abs(s) for s in samples)
        self.peak_display = val

        # First call: seed baseline
        if self.baseline is None:
            self.baseline = val
            return False

        if not self.in_spike:
            # Calm — update baseline slowly (only when not spiking)
            self.baseline = 0.96 * self.baseline + 0.04 * val

            deviation = val - self.baseline
            if deviation > RISE_THRESH and val > MIN_PEAK:
                # Rising edge detected → enter spike state
                self.in_spike = True
                self.peak     = val

        else:
            # Spiking — track the peak
            if val > self.peak:
                self.peak = val

            # Confirmed fall: val dropped to within FALL_FRAC of the rise range
            fall_target = self.baseline + (self.peak - self.baseline) * FALL_FRAC
            if val < fall_target:
                self.in_spike = False
                # Resume baseline tracking from current calm value
                self.baseline = 0.96 * self.baseline + 0.04 * val

                now = time.time()
                if self.peak > MIN_PEAK and (now - self.last_blink) > COOLDOWN:
                    self.count     += 1
                    self.last_blink = now
                    self.lit_until  = now + SHOW_MS / 1000.0
                    return True

        return False

    def is_lit(self):
        return time.time() < self.lit_until

# ── Per-eye detectors ─────────────────────────────────────────────────────────
left_det  = BlinkDetector()   # AF7 — left eye
right_det = BlinkDetector()   # AF8 — right eye
running   = True

def on_signal(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT,  on_signal)
signal.signal(signal.SIGTERM, on_signal)

# ── EEG callback (runs on Muse thread — keep it fast) ────────────────────────
# data[0]=TP9  data[1]=AF7  data[2]=AF8  data[3]=TP10
def eeg_callback(data, timestamps):
    left_det.process(data[1])
    right_det.process(data[2])

# ── Display ───────────────────────────────────────────────────────────────────
LINES       = 9
first_frame = True

def peak_bar(val, width=24):
    frac   = min(val / (MIN_PEAK * 2.5), 1.0)
    filled = int(frac * width)
    if val > MIN_PEAK:
        col = '\033[91m'   # red — above blink threshold
    elif val > MIN_PEAK * 0.5:
        col = '\033[93m'   # yellow — getting close
    else:
        col = '\033[92m'   # green — calm
    return col + '#' * filled + '\033[90m' + '-' * (width - filled) + '\033[0m'

def blink_tag(det):
    if det.is_lit():
        return '\033[1;97;44m  BLINK  \033[0m'
    return '\033[90m  ------  \033[0m'

def draw():
    global first_frame
    if not first_frame:
        sys.stdout.write(f'\033[{LINES}A\033[J')
    first_frame = False

    lv = left_det.peak_display
    rv = right_det.peak_display
    lb = left_det.baseline  if left_det.baseline  else 0.0
    rb = right_det.baseline if right_det.baseline else 0.0

    rows = [
        f'  Muse Blink Detector   [{time.strftime("%H:%M:%S")}]',
        f'',
        f'  LEFT  eye  AF7   val: {lv:6.0f} uV   base: {lb:5.0f}   [{peak_bar(lv)}]  {blink_tag(left_det)}',
        f'  RIGHT eye  AF8   val: {rv:6.0f} uV   base: {rb:5.0f}   [{peak_bar(rv)}]  {blink_tag(right_det)}',
        f'',
        f'  Blinks:  Left = {left_det.count:<6} Right = {right_det.count:<6}',
        f'',
        f'  Detection: spike must RISE > {RISE_THRESH} uV above baseline',
        f'             then FALL back down — sustained high = ignored',
    ]
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global running

    print('Scanning for Muse 2 (Bluetooth)...')
    found = find_muse(backend='bleak')
    if not found:
        print('No Muse device found. Make sure it is on and paired.')
        return

    address = found['address']
    name    = found.get('name', 'Muse')
    print(f'Found: {name}  ({address})\nConnecting...')

    muse = Muse(address=address, callback_eeg=eeg_callback, backend='bleak')
    if not muse.connect():
        print('Connection failed.')
        return

    muse.start()
    print(f'Connected. Detecting blinks on AF7 (left) and AF8 (right).\n')
    sys.stdout.write('\n' * LINES)

    t_alive = time.time()

    while running:
        now = time.time()

        if now - t_alive > 5:
            try:    muse.keep_alive()
            except: pass
            t_alive = now

        draw()
        time.sleep(0.05)   # 20 Hz display — detection is callback-driven (no delay)

    print('\nDisconnecting...')
    try:    muse.stop(); muse.disconnect()
    except: pass
    print('Done.')

if __name__ == '__main__':
    main()
