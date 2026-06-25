#!/usr/bin/env python3
"""
muse_athena_blink.py
Direct BLE connection to Muse S Athena — EEG streaming + per-eye blink detection.
Protocol reverse-engineered by amused-py (github.com/Amused-EEG/amused-py)

Run on Raspberry Pi. Pair the headset first:
  bluetoothctl -> pair 00:55:DA:B9:FC:10 -> trust -> quit
"""

import asyncio
import sys
import signal
import time
import threading
import numpy as np
from bleak import BleakClient, BleakScanner

# ── BLE UUIDs ─────────────────────────────────────────────────────────────────
MUSE_ADDR   = "00:55:DA:B9:FC:10"
CTRL_UUID   = "273e0001-4c4d-454d-96be-f03bac821358"
SENSOR_UUID = "273e0013-4c4d-454d-96be-f03bac821358"

# ── Athena packet protocol (from amused-py) ───────────────────────────────────
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

EEG_SCALE = 1450.0 / 16383.0   # 14-bit raw → µV

def encode_cmd(cmd: str) -> bytes:
    """Athena command encoding: [len+1] [cmd bytes] [newline]"""
    encoded = cmd.encode("utf-8") + b"\n"
    return bytes([len(encoded) + 1]) + encoded

# Init sequence — dc001 must be sent TWICE (once with p21, once with p1034)
INIT_SEQ = [
    ("v6",     encode_cmd("v6"),    0.05),
    ("s",      encode_cmd("s"),     0.05),
    ("h",      encode_cmd("h"),     0.10),
    ("p21",    encode_cmd("p21"),   0.05),
    ("s2",     encode_cmd("s"),     0.10),   # ← subscribe SENSOR_UUID after this
    ("dc001a", encode_cmd("dc001"), 0.05),
    ("L1a",    encode_cmd("L1"),    0.05),
    ("h2",     encode_cmd("h"),     0.10),
    ("p1034",  encode_cmd("p1034"), 0.05),
    ("s3",     encode_cmd("s"),     0.10),
    ("dc001b", encode_cmd("dc001"), 0.05),
    ("L1b",    encode_cmd("L1"),    0.10),
]
SUBSCRIBE_AFTER_STEP = "s2"


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
    for i in range(16):   # 4 samples × 4 channels = 16 values × 14 bits
        v = 0
        for b in range(14):
            if bits[i * 14 + b]:
                v |= (1 << b)
        raw.append(v)
    return np.array(raw, dtype=np.float32).reshape(4, 4) * EEG_SCALE


async def _find_muse_device(timeout: float = 12.0):
    print('Scanning for any Muse headset...')
    devices = await BleakScanner.discover(timeout=timeout)
    muse = next((d for d in devices if d.name and 'muse' in d.name.lower()), None)
    if muse is None:
        print(f'No Muse found. Seen: {[(d.address, d.name) for d in devices]}')
    return muse


def parse_payload(payload: bytes) -> list:
    """Parse TAG-based subpackets. Returns list of (tag, sensor_type, raw_bytes)."""
    results = []
    if len(payload) < HEADER_SIZE + 1:
        return results

    # First subpacket: TAG at header[9], data at byte 14
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

    # Additional subpackets: [TAG(1)] [header(4)] [data(N)]
    while offset + 5 < len(payload):
        tag = payload[offset]
        cfg = SENSOR_CONFIG.get(tag)
        if cfg is None:
            break
        data_len = cfg[3]
        data_start = offset + 5
        data_end   = data_start + data_len
        if data_end > len(payload):
            break
        results.append((tag, cfg[0], payload[data_start:data_end]))
        offset = data_end

    return results


# ── Blink detection ───────────────────────────────────────────────────────────
RISE_THRESH = 120    # µV above baseline to start spike
MIN_PEAK    = 180    # µV absolute minimum peak
FALL_FRAC   = 0.40   # must fall to this fraction of (peak-baseline)
COOLDOWN    = 0.35   # s  minimum gap between blinks
SHOW_MS     = 500    # ms keep BLINK indicator lit


class BlinkDetector:
    """Spike-pattern blink detector: rise → peak → fall. Sustained high = ignored."""

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
left_det   = BlinkDetector()   # AF7 = channel index 1
right_det  = BlinkDetector()   # AF8 = channel index 2
_eeg_vals  = [0.0, 0.0, 0.0, 0.0]   # TP9, AF7, AF8, TP10
_pkt_count = 0
running    = True


def on_sensor(handle, data: bytearray):
    global _pkt_count
    _pkt_count += 1
    subpackets = parse_payload(bytes(data))
    for tag, stype, raw in subpackets:
        if tag == 0x11:   # TAG_EEG_4CH
            arr = decode_eeg_4ch(raw)          # (4 samples, 4 channels)
            left_det.process(arr[:, 1])         # AF7 — left eye
            right_det.process(arr[:, 2])        # AF8 — right eye
            with _lock:
                for ch in range(4):
                    _eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))


def on_ctrl(handle, data: bytearray):
    pass   # JSON status responses — ignored during streaming


# ── Terminal display ──────────────────────────────────────────────────────────
LINES       = 11
first_frame = True


def eeg_bar(val: float, width: int = 22) -> str:
    frac   = min(val / 500.0, 1.0)
    filled = int(frac * width)
    if val > MIN_PEAK:
        col = '\033[91m'
    elif val > MIN_PEAK * 0.5:
        col = '\033[93m'
    else:
        col = '\033[92m'
    return col + '█' * filled + '\033[90m' + '░' * (width - filled) + '\033[0m'


def blink_tag(det: BlinkDetector) -> str:
    if det.is_lit():
        return '\033[1;97;44m BLINK \033[0m'
    return '\033[90m ------ \033[0m'


def draw():
    global first_frame
    if not first_frame:
        sys.stdout.write(f'\033[{LINES}A\033[J')
    first_frame = False

    with _lock:
        eeg = list(_eeg_vals)
    lv = left_det.peak_display
    rv = right_det.peak_display
    lb = left_det.baseline  or 0.0
    rb = right_det.baseline or 0.0

    rows = [
        f'  \033[1mMuse S Athena\033[0m — EEG + Blink Detector   [{time.strftime("%H:%M:%S")}]',
        f'  Packets: {_pkt_count}',
        f'',
        f'  \033[96m── EEG ──────────────────────────────────────────────────────────\033[0m',
        f'  TP9   {eeg[0]:7.1f} µV   {eeg_bar(eeg[0])}',
        f'  AF7   {lv:7.1f} µV   {eeg_bar(lv)}   {blink_tag(left_det)}  L:{left_det.count}  base:{lb:.0f}',
        f'  AF8   {rv:7.1f} µV   {eeg_bar(rv)}   {blink_tag(right_det)}  R:{right_det.count}  base:{rb:.0f}',
        f'  TP10  {eeg[3]:7.1f} µV   {eeg_bar(eeg[3])}',
        f'',
        f'  \033[90mBlink: spike >{RISE_THRESH}µV above baseline, peak >{MIN_PEAK}µV then fall\033[0m',
        f'  \033[90mCtrl-C to quit\033[0m',
    ]
    sys.stdout.write('\n'.join(rows) + '\n')
    sys.stdout.flush()


# ── Signal handler ────────────────────────────────────────────────────────────
def sig_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    global running

    device = await _find_muse_device()
    if device is None:
        print('No Muse headset found. Make sure the headset is on and retry.')
        return

    print(f"Connecting to Muse S Athena ({device.name} / {device.address})...")
    async with BleakClient(device, timeout=20.0) as client:
        print("Connected. Running Athena init sequence...")

        await client.start_notify(CTRL_UUID, on_ctrl)

        for step, cmd_bytes, delay in INIT_SEQ:
            await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
            await asyncio.sleep(delay)

            if step == SUBSCRIBE_AFTER_STEP:
                await client.start_notify(SENSOR_UUID, on_sensor)
                print("  Sensor notifications enabled")

        print("Init complete — waiting for EEG data...")
        await asyncio.sleep(2.0)

        if _pkt_count == 0:
            print("WARNING: No packets received yet — headset may need to be on your head.")

        print()
        sys.stdout.write('\n' * LINES)

        while running:
            draw()
            await asyncio.sleep(0.05)

    print("\nDisconnected.")


if __name__ == "__main__":
    asyncio.run(main())
