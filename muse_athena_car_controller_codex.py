#!/usr/bin/env python3
"""
muse_athena_car_controller_codex.py

Muse S / Muse 2 Athena EEG to Arduino car controller over HC-08 BLE.

Main controls:
  single blink      -> FORWARD, after the double-blink window closes
  double blink      -> STOP immediately on the second blink
  jaw clench        -> BACKWARD, after a short blink-rejection confirm window
  head roll left    -> curve left while moving
  head roll right   -> curve right while moving

Safety controls:
  SPACE             -> emergency stop latch / re-arm
  r                 -> recalibrate EEG, jaw, and roll offset
  q                 -> quit safely

Run:
  .\\.venv\\Scripts\\python.exe .\\muse_athena_car_controller_codex.py

Safe check:
  .\\.venv\\Scripts\\python.exe .\\muse_athena_car_controller_codex.py --check

This file keeps the original tuned signal behavior, but reorganizes the script
around explicit state objects, stronger BLE reconnect handling, a stale-stream
watchdog, STOP heartbeats, and a cleaner terminal dashboard.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import random
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

try:
    import msvcrt  # Windows non-blocking keyboard input.
except ImportError:  # pragma: no cover - this project runs on Windows.
    msvcrt = None

import numpy as np

try:
    import bleak
    from bleak import BleakClient, BleakScanner
except ImportError:  # Allows --check to explain missing deps cleanly.
    bleak = None
    BleakClient = None
    BleakScanner = None

from config import HC08_ADDRESS, UART_CHAR_UUID, DEFAULT_SPEED, ROLL_THRESHOLD


# ---------------------------------------------------------------------------
# Athena BLE protocol and tuned controller constants
# ---------------------------------------------------------------------------

MUSE_NAME_HINT = "muse"
MUSE_PREFERRED_ADDRESS = "00:55:DA:B9:FC:10"
CTRL_UUID = "273e0001-4c4d-454d-96be-f03bac821358"
SENSOR_UUID = "273e0013-4c4d-454d-96be-f03bac821358"

HEADER_SIZE = 14
SENSOR_CONFIG = {
    0x11: ("EEG", 4, 4, 28),
    0x12: ("EEG", 8, 2, 28),
    0x34: ("OPTICS", 4, 3, 30),
    0x35: ("OPTICS", 8, 2, 40),
    0x36: ("OPTICS", 16, 1, 40),
    0x47: ("ACCGYRO", 6, 3, 36),
    0x88: ("BATTERY", 1, 1, 188),
    0x98: ("BATTERY", 1, 1, 20),
}

EEG_SCALE = 1450.0 / 16383.0
ACC_SCALE = 0.0000610352
GYRO_SCALE = -0.0074768

SYNC_DURATION = 30.0

RISE_THRESH = 144.0
MIN_PEAK = 216.0
LEFT_RISE_THRESH = 77.0
LEFT_MIN_PEAK = 115.0
FALL_FRAC = 0.40
COOLDOWN = 0.20
SHOW_MS = 500
SATURATION = 1430.0

BLINK_MERGE = 0.18
MULTI_WINDOW = 1.00

JAW_WIN = 64
JAW_K = 3.375
JAW_TP9_SCALE = 1.20
JAW_REARM_FRAC = 0.65
JAW_REQUIRE_BOTH_CHANNELS = False
JAW_SPREAD_MIN = 0.08
JAW_HIST_LEN = 240
JAW_COOLDOWN = 1.0
JAW_CONFIRM_DELAY = 0.25

EEG_WATCHDOG_TIMEOUT = 0.6
IMU_STALE_TIMEOUT = 2.0
STREAM_STALE_TIMEOUT = 3.0
STOP_HEARTBEAT = 0.25
MOTION_HEARTBEAT = 0.35

CONTROL_HZ = 50.0
DRAW_HZ = 10.0
CAR_POLL_SECONDS = 0.01

SCAN_TIMEOUT = 12.0
CONNECT_TIMEOUT = 20.0
NOTIFY_ATTEMPTS = 5
BACKOFF_BASE = 1.5
BACKOFF_CAP = 20.0

DRIVE_STOP = 0
DRIVE_FORWARD = 1
DRIVE_BACKWARD = 2
DRIVE_LABELS = ("STOP", "FORWARD", "BACKWARD")

CMD_DESC = {
    "F": "FORWARD",
    "B": "BACKWARD",
    "Q": "FWD LEFT",
    "E": "FWD RIGHT",
    "G": "BACK LEFT",
    "H": "BACK RIGHT",
    "S": "STOP",
}


def encode_cmd(cmd: str) -> bytes:
    """Wrap a Muse text command as length-prefixed UTF-8 plus newline."""
    encoded = cmd.encode("utf-8") + b"\n"
    return bytes([len(encoded) + 1]) + encoded


INIT_SEQ = [
    ("v6", encode_cmd("v6"), 0.05),
    ("s", encode_cmd("s"), 0.05),
    ("h", encode_cmd("h"), 0.10),
    ("p21", encode_cmd("p21"), 0.05),
    ("s2", encode_cmd("s"), 0.10),
    ("dc001a", encode_cmd("dc001"), 0.05),
    ("L1a", encode_cmd("L1"), 0.05),
    ("h2", encode_cmd("h"), 0.10),
    ("p1034", encode_cmd("p1034"), 0.05),
    ("s3", encode_cmd("s"), 0.10),
    ("dc001b", encode_cmd("dc001"), 0.05),
    ("L1b", encode_cmd("L1"), 0.10),
]
SUBSCRIBE_AFTER_STEP = "s2"


# ---------------------------------------------------------------------------
# Terminal helpers
# ---------------------------------------------------------------------------

RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
GREY = "\033[90m"
INV_RED = "\033[1;97;41m"
INV_BLUE = "\033[1;97;44m"


def color(text: str, code: str) -> str:
    return f"{code}{text}{RESET}"


def enable_windows_terminal_colors() -> None:
    if sys.platform != "win32":
        return
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except Exception:
        pass
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Pure decoders
# ---------------------------------------------------------------------------

def decode_eeg_4ch(data: bytes) -> np.ndarray:
    """Decode 14-bit LSB-first Athena EEG into (4 samples, 4 channels) uV."""
    bits = np.unpackbits(np.frombuffer(data[:28], dtype=np.uint8), bitorder="little")
    weights = (1 << np.arange(14, dtype=np.uint32))
    raw = bits[:224].reshape(16, 14).astype(np.uint32) @ weights
    return raw.astype(np.float32).reshape(4, 4) * EEG_SCALE


def decode_accgyro(data: bytes) -> np.ndarray:
    """Decode Athena ACC/GYRO into acc g and gyro deg/s, shape (3, 6)."""
    raw = np.frombuffer(data[:36], dtype="<i2").reshape(3, 6).astype(np.float32)
    result = raw.copy()
    result[:, 0:3] *= ACC_SCALE
    result[:, 3:6] *= GYRO_SCALE
    return result


def angle_delta_deg(angle: float, reference: float) -> float:
    """Shortest signed difference between two angles, normalized to -180..+180.

    Without this, crossing the -180/+180 boundary can turn a real left tilt into
    a fake +300 degree right tilt.
    """
    return ((angle - reference + 180.0) % 360.0) - 180.0


def parse_payload(payload: bytes) -> list[tuple[int, str, bytes]]:
    """Split one Athena notification into recognized sub-packets."""
    packets: list[tuple[int, str, bytes]] = []
    if len(payload) < HEADER_SIZE + 1:
        return packets

    tag = payload[9]
    cfg = SENSOR_CONFIG.get(tag)
    if cfg is None:
        return packets

    data_len = cfg[3]
    data_end = HEADER_SIZE + data_len
    if data_end > len(payload):
        return packets
    packets.append((tag, cfg[0], payload[HEADER_SIZE:data_end]))

    offset = data_end
    while offset + 5 < len(payload):
        tag = payload[offset]
        cfg = SENSOR_CONFIG.get(tag)
        if cfg is None:
            break
        data_len = cfg[3]
        start = offset + 5
        end = start + data_len
        if end > len(payload):
            break
        packets.append((tag, cfg[0], payload[start:end]))
        offset = end
    return packets


# ---------------------------------------------------------------------------
# Signal detectors
# ---------------------------------------------------------------------------

class BlinkDetector:
    """Original tuned rise/fall blink detector for one eye channel."""

    def __init__(self, rise_thresh: float, min_peak: float):
        self.rise_thresh = rise_thresh
        self.min_peak = min_peak
        self.baseline: Optional[float] = None
        self.in_spike = False
        self.peak = 0.0
        self.count = 0
        self.last_blink = 0.0
        self.lit_until = 0.0
        self.peak_display = 0.0
        self.saturated = False

    def process(self, samples: np.ndarray, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else now
        val = float(np.max(np.abs(samples)))
        self.peak_display = val
        self.saturated = val >= SATURATION

        if self.baseline is None:
            self.baseline = val
            return False

        if self.saturated:
            self.in_spike = False
            self.baseline = 0.96 * self.baseline + 0.04 * val
            return False

        if not self.in_spike:
            self.baseline = 0.96 * self.baseline + 0.04 * val
            if (val - self.baseline) > self.rise_thresh and val > self.min_peak:
                self.in_spike = True
                self.peak = val
        else:
            self.peak = max(self.peak, val)
            fall_target = self.baseline + (self.peak - self.baseline) * FALL_FRAC
            if val < fall_target:
                self.in_spike = False
                self.baseline = 0.96 * self.baseline + 0.04 * val
                if self.peak > self.min_peak and (now - self.last_blink) > COOLDOWN:
                    self.count += 1
                    self.last_blink = now
                    self.lit_until = now + SHOW_MS / 1000.0
                    return True
        return False

    def is_lit(self) -> bool:
        return time.time() < self.lit_until


def _mad(arr: np.ndarray) -> float:
    med = np.median(arr)
    return float(np.median(np.abs(arr - med)) * 1.4826)


def _jaw_trigger(base: float, spread: float) -> float:
    spread = max(spread, JAW_SPREAD_MIN * base)
    return base + JAW_K * spread


def _scaled_trigger(base: float, trigger: float, scale: float) -> float:
    return base + (trigger - base) * scale


class JawDetector:
    """Adaptive jaw clench detector.

    TP9 is the primary trigger because this headset session has TP10 pinned high.
    TP10 is still measured for the dashboard, but it no longer blocks BACKWARD.
    """

    def __init__(self):
        self.tp9_buf: deque[float] = deque(maxlen=JAW_WIN)
        self.tp10_buf: deque[float] = deque(maxlen=JAW_WIN)
        self.emg9_hist: deque[float] = deque([10.0] * JAW_HIST_LEN, maxlen=JAW_HIST_LEN)
        self.emg10_hist: deque[float] = deque([10.0] * JAW_HIST_LEN, maxlen=JAW_HIST_LEN)
        self.emg9 = 0.0
        self.emg10 = 0.0
        self.emg9_base = 10.0
        self.emg10_base = 10.0
        self.emg9_trig = 0.0
        self.emg10_trig = 0.0
        self.emg9_rearm = 0.0
        self.tp9_over = False
        self.tp10_over = False
        self.jaw_armed = True
        self.last_jaw = 0.0
        self.count = 0
        self.lit_until = 0.0

    def process(self, arr: np.ndarray, calibrating: bool, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else now
        self.tp9_buf.extend(arr[:, 0].tolist())
        self.tp10_buf.extend(arr[:, 3].tolist())
        if len(self.tp9_buf) < 16:
            return False

        a9 = np.array(self.tp9_buf)
        a10 = np.array(self.tp10_buf)
        self.emg9 = float(np.mean(np.abs(np.diff(a9))))
        self.emg10 = float(np.mean(np.abs(np.diff(a10))))

        self.emg9_base = float(np.median(self.emg9_hist))
        self.emg10_base = float(np.median(self.emg10_hist))
        emg9_raw_trig = _jaw_trigger(self.emg9_base, _mad(np.array(self.emg9_hist)))
        self.emg9_trig = _scaled_trigger(self.emg9_base, emg9_raw_trig, JAW_TP9_SCALE)
        self.emg9_rearm = _scaled_trigger(self.emg9_base, self.emg9_trig, JAW_REARM_FRAC)
        self.emg10_trig = _jaw_trigger(self.emg10_base, _mad(np.array(self.emg10_hist)))

        self.tp9_over = self.emg9 > self.emg9_trig
        self.tp10_over = self.emg10 > self.emg10_trig
        jaw_ready = (
            self.jaw_armed
            and self.tp9_over
            and (self.tp10_over or not JAW_REQUIRE_BOTH_CHANNELS)
        )

        if calibrating or not self.tp9_over:
            self.emg9_hist.append(self.emg9)
        if calibrating or not self.tp10_over:
            self.emg10_hist.append(self.emg10)

        if calibrating:
            self.jaw_armed = not self.tp9_over
            return False
        if self.emg9 <= self.emg9_rearm:
            self.jaw_armed = True

        if (
            jaw_ready
            and now - self.last_jaw > JAW_COOLDOWN
        ):
            self.jaw_armed = False
            self.last_jaw = now
            self.count += 1
            self.lit_until = now + 0.8
            return True
        return False

    def is_lit(self) -> bool:
        return time.time() < self.lit_until


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

@dataclass
class DriveState:
    mode: int = DRIVE_STOP
    estop: bool = False
    last_command: str = "S"
    last_send_ts: float = 0.0
    last_stop_reason: str = "startup"


@dataclass
class ImuState:
    roll_raw: float = 0.0
    roll: float = 0.0
    roll_offset: float = 0.0
    roll_samples: list[float] = field(default_factory=list)
    calibrated: bool = False
    ts: float = 0.0


@dataclass
class BlinkBurst:
    last_event: float = 0.0
    burst_count: int = 0
    burst_last: float = 0.0
    stop_inhibit_until: float = 0.0
    total_events: int = 0
    last_action: str = "-"


@dataclass
class JawPending:
    active: bool = False
    ts: float = 0.0
    last_blink_fire: float = 0.0


@dataclass
class ConnectionState:
    muse_connected: bool = False
    muse_status: str = "Waiting..."
    hc08_connected: bool = False
    hc08_status: str = "Waiting..."
    packet_count: int = 0
    last_packet_ts: float = 0.0
    last_eeg_ts: float = 0.0


@dataclass
class SensorState:
    eeg_vals: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    sync_start: float = float("inf")
    reset_requested: bool = False


@dataclass
class AppState:
    left_eye: BlinkDetector = field(default_factory=lambda: BlinkDetector(LEFT_RISE_THRESH, LEFT_MIN_PEAK))
    right_eye: BlinkDetector = field(default_factory=lambda: BlinkDetector(RISE_THRESH, MIN_PEAK))
    jaw: JawDetector = field(default_factory=JawDetector)
    drive: DriveState = field(default_factory=DriveState)
    imu: ImuState = field(default_factory=ImuState)
    blink: BlinkBurst = field(default_factory=BlinkBurst)
    jaw_pending: JawPending = field(default_factory=JawPending)
    conn: ConnectionState = field(default_factory=ConnectionState)
    sensor: SensorState = field(default_factory=SensorState)


STATE = AppState()
STATE_LOCK = threading.RLock()
STOP_EVENT = threading.Event()


@dataclass
class CarCommandState:
    desired: str = "S"
    dirty: bool = True


CAR_CMD = CarCommandState()
CAR_LOCK = threading.Lock()


def is_syncing(now: Optional[float] = None) -> bool:
    now = time.time() if now is None else now
    with STATE_LOCK:
        return (now - STATE.sensor.sync_start) < SYNC_DURATION


def sync_remaining(now: Optional[float] = None) -> float:
    now = time.time() if now is None else now
    with STATE_LOCK:
        return max(0.0, SYNC_DURATION - (now - STATE.sensor.sync_start))


def request_car_cmd(cmd: str, force: bool = False) -> None:
    """Publish the latest desired Arduino command.

    The car thread reads this single latest value instead of a growing queue.
    That avoids old FORWARD commands being replayed after a reconnect.
    """
    with CAR_LOCK:
        if force or cmd != CAR_CMD.desired:
            CAR_CMD.desired = cmd
            CAR_CMD.dirty = True


def force_stop(reason: str = "", immediate: bool = False) -> None:
    with STATE_LOCK:
        STATE.drive.mode = DRIVE_STOP
        if reason:
            STATE.drive.last_stop_reason = reason
    request_car_cmd("S", force=immediate)


def _take_next_car_cmd(now: float) -> Optional[str]:
    with STATE_LOCK:
        elapsed = now - STATE.drive.last_send_ts
    with CAR_LOCK:
        desired = CAR_CMD.desired
        heartbeat = STOP_HEARTBEAT if desired == "S" else MOTION_HEARTBEAT
        if CAR_CMD.dirty or elapsed >= heartbeat:
            CAR_CMD.dirty = False
            return desired
    return None


# ---------------------------------------------------------------------------
# HC-08 car BLE link
# ---------------------------------------------------------------------------

async def _write_car_char(client, data: bytes) -> None:
    """Prefer acknowledged writes, then fall back for modules that reject them."""
    try:
        await client.write_gatt_char(UART_CHAR_UUID, data, response=True)
    except Exception:
        await client.write_gatt_char(UART_CHAR_UUID, data, response=False)


def _opposite_motion(a: str, b: str) -> bool:
    forward = {"F", "Q", "E"}
    backward = {"B", "G", "H"}
    return (a in forward and b in backward) or (a in backward and b in forward)


async def car_ble_main() -> None:
    if bleak is None or BleakClient is None:
        with STATE_LOCK:
            STATE.conn.hc08_status = "Bleak not installed"
        return

    attempt = 0
    while not STOP_EVENT.is_set():
        attempt += 1
        with STATE_LOCK:
            STATE.conn.hc08_connected = False
            STATE.conn.hc08_status = f"Connecting attempt {attempt}"
        try:
            async with BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                attempt = 0
                with STATE_LOCK:
                    STATE.conn.hc08_connected = True
                    STATE.conn.hc08_status = f"Connected {HC08_ADDRESS}"

                request_car_cmd("S", force=True)
                await _write_car_char(client, b"S")
                await _write_car_char(client, DEFAULT_SPEED.encode("ascii"))

                while client.is_connected and not STOP_EVENT.is_set():
                    now = time.time()
                    cmd = _take_next_car_cmd(now)
                    if cmd is not None:
                        with STATE_LOCK:
                            previous = STATE.drive.last_command
                        if _opposite_motion(previous, cmd):
                            await _write_car_char(client, b"S")
                            await asyncio.sleep(0.08)
                        await _write_car_char(client, cmd.encode("ascii"))
                        with STATE_LOCK:
                            STATE.drive.last_command = cmd
                            STATE.drive.last_send_ts = now
                    await asyncio.sleep(CAR_POLL_SECONDS)

                try:
                    await _write_car_char(client, b"S")
                except Exception:
                    pass
        except Exception as exc:
            with STATE_LOCK:
                STATE.conn.hc08_status = f"Retrying ({type(exc).__name__})"
        finally:
            with STATE_LOCK:
                STATE.conn.hc08_connected = False
            force_stop("hc08 disconnected", immediate=True)

        if not STOP_EVENT.is_set():
            await asyncio.sleep(backoff(attempt))


def car_thread() -> None:
    try:
        asyncio.run(car_ble_main())
    except Exception as exc:
        with STATE_LOCK:
            STATE.conn.hc08_connected = False
            STATE.conn.hc08_status = f"Car thread stopped: {type(exc).__name__}"


# ---------------------------------------------------------------------------
# Blink, jaw, IMU, and drive control
# ---------------------------------------------------------------------------

def backoff(attempt: int) -> float:
    raw = min(BACKOFF_CAP, BACKOFF_BASE * (2 ** max(0, attempt - 1)))
    return raw + random.uniform(0.0, min(1.0, raw * 0.25))


def reset_calibration() -> None:
    with STATE_LOCK:
        STATE.left_eye = BlinkDetector(LEFT_RISE_THRESH, LEFT_MIN_PEAK)
        STATE.right_eye = BlinkDetector(RISE_THRESH, MIN_PEAK)
        STATE.jaw = JawDetector()
        STATE.imu = ImuState()
        STATE.blink = BlinkBurst()
        STATE.jaw_pending = JawPending()
        STATE.drive.mode = DRIVE_STOP
        STATE.sensor.sync_start = time.time()
        STATE.sensor.reset_requested = False
    force_stop("recalibrate", immediate=True)


def register_blink(now: float) -> None:
    blink = STATE.blink
    pending = STATE.jaw_pending
    if now < blink.stop_inhibit_until:
        return
    if now - blink.last_event < BLINK_MERGE:
        return

    blink.last_event = now
    blink.total_events += 1
    pending.last_blink_fire = now

    if pending.active and (now - pending.ts) < JAW_CONFIRM_DELAY:
        pending.active = False

    if blink.burst_count > 0 and (now - blink.burst_last) <= MULTI_WINDOW:
        blink.burst_count += 1
    else:
        blink.burst_count = 1
    blink.burst_last = now


def resolve_blinks(now: float) -> tuple[bool, bool]:
    with STATE_LOCK:
        blink = STATE.blink
        if blink.burst_count >= 2:
            blink.last_action = f"STOP({blink.burst_count})"
            blink.burst_count = 0
            blink.stop_inhibit_until = now + MULTI_WINDOW
            return False, True
        if blink.burst_count == 1 and (now - blink.burst_last) > MULTI_WINDOW:
            blink.last_action = "FWD(1)"
            blink.burst_count = 0
            return True, False
    return False, False


def on_sensor(handle, data: bytearray) -> None:
    now = time.time()
    payload = bytes(data)
    subpackets = parse_payload(payload)
    if not subpackets:
        return

    with STATE_LOCK:
        STATE.conn.packet_count += 1
        STATE.conn.last_packet_ts = now

    for tag, _stype, raw in subpackets:
        if tag == 0x11:
            arr = decode_eeg_4ch(raw)
            with STATE_LOCK:
                syncing = is_syncing(now)
                blink_l = STATE.left_eye.process(arr[:, 1], now)
                blink_r = STATE.right_eye.process(arr[:, 2], now)
                jaw_fired = STATE.jaw.process(arr, syncing, now)
                STATE.conn.last_eeg_ts = now
                for ch in range(4):
                    STATE.sensor.eeg_vals[ch] = float(np.max(np.abs(arr[:, ch])))

                if (blink_l or blink_r) and not syncing:
                    register_blink(now)

                if jaw_fired and not syncing:
                    STATE.jaw_pending.active = True
                    STATE.jaw_pending.ts = now

        elif tag == 0x47:
            imu_arr = decode_accgyro(raw)
            ay = float(imu_arr[-1, 1])
            az = float(imu_arr[-1, 2])
            roll_raw = math.degrees(math.atan2(ay, az))

            with STATE_LOCK:
                STATE.imu.roll_raw = roll_raw
                STATE.imu.ts = now
                if is_syncing(now):
                    STATE.imu.roll_samples.append(roll_raw)
                else:
                    if not STATE.imu.calibrated and STATE.imu.roll_samples:
                        STATE.imu.roll_offset = float(np.median(STATE.imu.roll_samples))
                        STATE.imu.calibrated = True
                    STATE.imu.roll = angle_delta_deg(roll_raw, STATE.imu.roll_offset)


def on_ctrl(handle, data: bytearray) -> None:
    # Control notifications are useful for debug, but not needed for decisions.
    return


def update_control() -> None:
    now = time.time()
    forward, stop = resolve_blinks(now)

    jaw_back = False
    with STATE_LOCK:
        pending = STATE.jaw_pending
        if pending.active and (now - pending.ts) >= JAW_CONFIRM_DELAY:
            pending.active = False
            if pending.ts - pending.last_blink_fire >= JAW_CONFIRM_DELAY:
                jaw_back = True

        if jaw_back:
            STATE.drive.mode = DRIVE_BACKWARD
        if forward:
            STATE.drive.mode = DRIVE_FORWARD
        if stop:
            STATE.drive.mode = DRIVE_STOP

        connected = STATE.conn.muse_connected
        car_connected = STATE.conn.hc08_connected
        syncing = is_syncing(now)
        estop = STATE.drive.estop
        eeg_fresh = STATE.conn.last_eeg_ts > 0 and (now - STATE.conn.last_eeg_ts) <= EEG_WATCHDOG_TIMEOUT
        contact_bad = STATE.left_eye.saturated or STATE.right_eye.saturated
        drive_mode = STATE.drive.mode
        roll = STATE.imu.roll
        has_imu = STATE.imu.ts > 0 and (now - STATE.imu.ts) <= IMU_STALE_TIMEOUT

    if estop or not connected or not car_connected or syncing or not eeg_fresh or contact_bad:
        force_stop("watchdog")
        return

    left = has_imu and roll < -ROLL_THRESHOLD
    right = has_imu and roll > ROLL_THRESHOLD

    if drive_mode == DRIVE_FORWARD:
        if left:
            request_car_cmd("Q")
        elif right:
            request_car_cmd("E")
        else:
            request_car_cmd("F")
    elif drive_mode == DRIVE_BACKWARD:
        if left:
            request_car_cmd("G")
        elif right:
            request_car_cmd("H")
        else:
            request_car_cmd("B")
    else:
        request_car_cmd("S", force=True)


# ---------------------------------------------------------------------------
# Muse BLE link
# ---------------------------------------------------------------------------

def log_debug(msg: str) -> None:
    try:
        with open("muse_debug.log", "a", encoding="utf-8") as fh:
            fh.write(f"{time.strftime('%H:%M:%S')} {msg}\n")
    except Exception:
        pass


async def notify_with_retry(client, uuid: str, callback: Callable, label: str) -> None:
    for attempt in range(1, NOTIFY_ATTEMPTS + 1):
        try:
            await client.start_notify(uuid, callback)
            log_debug(f"notify {label} ok attempt {attempt}")
            return
        except Exception as exc:
            log_debug(f"notify {label} failed attempt {attempt}: {type(exc).__name__}: {exc}")
            if attempt == NOTIFY_ATTEMPTS:
                raise
            await asyncio.sleep(0.35 + attempt * 0.15)


async def find_muse_device():
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    by_address = next(
        (d for d in devices if d.address and d.address.upper() == MUSE_PREFERRED_ADDRESS.upper()),
        None,
    )
    if by_address is not None:
        return by_address, devices
    by_name = next(
        (d for d in devices if d.name and MUSE_NAME_HINT in d.name.lower()),
        None,
    )
    return by_name, devices


async def muse_main() -> None:
    if bleak is None or BleakClient is None or BleakScanner is None:
        with STATE_LOCK:
            STATE.conn.muse_status = "Bleak not installed"
        return

    attempt = 0
    while not STOP_EVENT.is_set():
        attempt += 1
        with STATE_LOCK:
            STATE.conn.muse_connected = False
            STATE.conn.muse_status = f"Scanning attempt {attempt}"

        try:
            device, devices = await find_muse_device()
            if device is None:
                names = [(d.address, d.name) for d in devices]
                log_debug(f"no muse found; saw {names}")
                with STATE_LOCK:
                    STATE.conn.muse_status = "No Muse found; retrying"
                await asyncio.sleep(backoff(attempt))
                continue

            with STATE_LOCK:
                STATE.conn.muse_status = f"Found {device.name or 'Muse'}; connecting"
            log_debug(f"found {device.name} {device.address}; connecting")

            def disconnected_callback(_client) -> None:
                with STATE_LOCK:
                    STATE.conn.muse_connected = False
                    STATE.conn.muse_status = "Disconnected; retrying"
                force_stop("muse disconnected", immediate=True)

            async with BleakClient(
                device,
                timeout=CONNECT_TIMEOUT,
                disconnected_callback=disconnected_callback,
            ) as client:
                services = client.services
                ctrl = services.get_characteristic(CTRL_UUID)
                sensor = services.get_characteristic(SENSOR_UUID)
                if ctrl is None or sensor is None:
                    with STATE_LOCK:
                        STATE.conn.muse_status = "GATT missing; reconnecting"
                    log_debug("GATT missing CTRL/SENSOR characteristic")
                    await asyncio.sleep(1.0)
                    continue

                await notify_with_retry(client, CTRL_UUID, on_ctrl, "CTRL")

                for step, cmd_bytes, delay in INIT_SEQ:
                    await client.write_gatt_char(CTRL_UUID, cmd_bytes, response=False)
                    await asyncio.sleep(delay)
                    if step == SUBSCRIBE_AFTER_STEP:
                        await notify_with_retry(client, SENSOR_UUID, on_sensor, "SENSOR")

                now = time.time()
                with STATE_LOCK:
                    STATE.drive.mode = DRIVE_STOP
                    STATE.blink = BlinkBurst()
                    STATE.jaw_pending = JawPending()
                    STATE.sensor.sync_start = now
                    STATE.imu = ImuState()
                    STATE.conn.muse_connected = True
                    STATE.conn.muse_status = f"Streaming {device.name or device.address}"
                    STATE.conn.last_packet_ts = now
                    STATE.conn.last_eeg_ts = 0.0
                request_car_cmd("S", force=True)
                attempt = 0
                log_debug("muse init complete")

                while client.is_connected and not STOP_EVENT.is_set():
                    await asyncio.sleep(0.2)
                    with STATE_LOCK:
                        last_packet = STATE.conn.last_packet_ts
                    if last_packet and time.time() - last_packet > STREAM_STALE_TIMEOUT:
                        with STATE_LOCK:
                            STATE.conn.muse_status = "Stream stalled; reconnecting"
                        force_stop("stream stalled", immediate=True)
                        log_debug("stream stalled; reconnect")
                        break

        except Exception as exc:
            log_debug(f"muse exception {type(exc).__name__}: {exc}")
            with STATE_LOCK:
                STATE.conn.muse_status = f"Retrying ({type(exc).__name__})"
        finally:
            with STATE_LOCK:
                STATE.conn.muse_connected = False
            force_stop("muse loop finally", immediate=True)

        if not STOP_EVENT.is_set():
            await asyncio.sleep(backoff(attempt))


# ---------------------------------------------------------------------------
# Dashboard and keyboard
# ---------------------------------------------------------------------------

def bar(value: float, full: float, width: int = 22) -> str:
    frac = max(0.0, min(value / full, 1.0))
    filled = int(frac * width)
    code = RED if frac >= 0.9 else (YELLOW if frac >= 0.5 else GREEN)
    return color("#" * filled, code) + color("-" * (width - filled), GREY)


def frac_bar(frac: float, width: int = 12) -> str:
    frac = max(0.0, min(frac, 1.0))
    filled = int(frac * width)
    code = RED if frac >= 1.0 else (YELLOW if frac > 0.6 else GREEN)
    return color("#" * filled, code) + color("-" * (width - filled), GREY)


def blink_tag(det: BlinkDetector) -> str:
    if det.saturated:
        return color(" NO CONTACT ", INV_RED)
    if det.is_lit():
        return color(" BLINK ", INV_BLUE)
    return color(" ------ ", GREY)


def conn_cell(connected: bool, status: str) -> str:
    if connected:
        return f"{color('CONNECTED', GREEN)} {status}"
    return f"{color('waiting', RED)} {status}"


def roll_text(imu: ImuState) -> str:
    now = time.time()
    if imu.ts <= 0 or now - imu.ts > IMU_STALE_TIMEOUT:
        return color("IMU stale/no data", GREY)
    if imu.roll < -ROLL_THRESHOLD:
        direction = color("TURN LEFT", CYAN)
    elif imu.roll > ROLL_THRESHOLD:
        direction = color("TURN RIGHT", CYAN)
    else:
        direction = color("STRAIGHT", GREEN)
    return (
        f"roll {imu.roll:+6.1f} deg  raw {imu.roll_raw:+6.1f}  "
        f"zero {imu.roll_offset:+6.1f}  {direction}"
    )


def jaw_text(jaw: JawDetector) -> str:
    f9 = (jaw.emg9 - jaw.emg9_base) / (jaw.emg9_trig - jaw.emg9_base) if jaw.emg9_trig > jaw.emg9_base else 0.0
    f10 = (jaw.emg10 - jaw.emg10_base) / (jaw.emg10_trig - jaw.emg10_base) if jaw.emg10_trig > jaw.emg10_base else 0.0
    tag = color(" JAW! ", INV_RED) if jaw.is_lit() else color(" ---- ", GREY)
    mode = "TP9-only" if not JAW_REQUIRE_BOTH_CHANNELS else "TP9+TP10"
    tp9 = color("TP9*", GREEN) if jaw.tp9_over else "TP9 "
    tp10 = color("TP10*", GREEN) if jaw.tp10_over else "TP10 "
    armed = color("armed", GREEN) if jaw.jaw_armed else color("wait-release", YELLOW)
    return (
        f"{tp9} {frac_bar(f9)}  {tp10} {frac_bar(f10)}  "
        f"{tag} count:{jaw.count} mode:{mode} {armed}"
    )


def draw_dashboard(previous_lines: int) -> int:
    if previous_lines > 0:
        sys.stdout.write(f"\033[{previous_lines}A\033[J")

    with STATE_LOCK:
        eeg = list(STATE.sensor.eeg_vals)
        left_eye = STATE.left_eye
        right_eye = STATE.right_eye
        jaw = STATE.jaw
        drive = DriveState(**STATE.drive.__dict__)
        imu = ImuState(
            roll_raw=STATE.imu.roll_raw,
            roll=STATE.imu.roll,
            roll_offset=STATE.imu.roll_offset,
            roll_samples=[],
            calibrated=STATE.imu.calibrated,
            ts=STATE.imu.ts,
        )
        blink = BlinkBurst(**STATE.blink.__dict__)
        conn = ConnectionState(**STATE.conn.__dict__)
        syncing = is_syncing()
        remaining = sync_remaining()

    if drive.estop:
        banner = color(" -- EMERGENCY STOP - press SPACE to re-arm -- ", INV_RED)
    elif not conn.muse_connected:
        banner = color(" -- WAITING FOR MUSE HEADSET -- ", RED)
    elif syncing:
        banner = color(f" -- SYNCING {remaining:4.0f}s - keep headset still -- ", YELLOW)
    else:
        banner = color(" -- ACTIVE -- ", GREEN)

    drive_label = DRIVE_LABELS[drive.mode]
    if drive.estop or not conn.muse_connected or syncing:
        drive_line = f"Drive: {color(drive_label, YELLOW)} {color('(not active)', GREY)}"
    else:
        drive_line = f"Drive: {color(drive_label, GREEN)}  blink=FWD  jaw=BACK  2 blinks=STOP"

    now = time.strftime("%H:%M:%S")
    rows = [
        f"  {color('Muse Athena Car Controller - Codex', BOLD)}  [{now}]",
        f"  Muse   {conn_cell(conn.muse_connected, conn.muse_status)}  packets:{conn.packet_count}",
        f"  HC-08  {conn_cell(conn.hc08_connected, conn.hc08_status)}",
        "",
        f"  {banner}",
        "",
        f"  {drive_line}",
        f"  Steer: {roll_text(imu)}",
        f"  CMD:   {CMD_DESC.get(drive.last_command, drive.last_command)}"
        f"   {color('stop reason: ' + drive.last_stop_reason, GREY)}",
        "",
        f"  {color('-- EEG --', CYAN)}",
        f"  TP9   {eeg[0]:7.1f} uV  {bar(eeg[0], 500.0)}",
        f"  AF7   {left_eye.peak_display:7.1f} uV  {bar(left_eye.peak_display, 500.0)}  {blink_tag(left_eye)} L:{left_eye.count} base:{(left_eye.baseline or 0.0):.0f}",
        f"  AF8   {right_eye.peak_display:7.1f} uV  {bar(right_eye.peak_display, 500.0)}  {blink_tag(right_eye)} R:{right_eye.count} base:{(right_eye.baseline or 0.0):.0f}",
        f"  TP10  {eeg[3]:7.1f} uV  {bar(eeg[3], 500.0)}",
        "",
        f"  {color('-- JAW --', CYAN)}",
        f"  {jaw_text(jaw)}",
        "",
        f"  blinks: burst={blink.burst_count} total={blink.total_events} last={blink.last_action}",
        f"  {color('Ctrl-C/q quit | SPACE emergency stop | r recalibrate', GREY)}",
    ]
    sys.stdout.write("\n".join(rows) + "\n")
    sys.stdout.flush()
    return len(rows)


def handle_key(ch: str) -> None:
    if ch in ("q", "Q", "\x03"):
        STOP_EVENT.set()
        force_stop("quit", immediate=True)
    elif ch in ("r", "R"):
        reset_calibration()
    elif ch == " ":
        with STATE_LOCK:
            STATE.drive.estop = not STATE.drive.estop
            if STATE.drive.estop:
                STATE.drive.mode = DRIVE_STOP
        force_stop("estop", immediate=True)


def ui_thread() -> None:
    previous_lines = 0
    next_control = time.perf_counter()
    next_draw = time.perf_counter()
    control_period = 1.0 / CONTROL_HZ
    draw_period = 1.0 / DRAW_HZ

    while not STOP_EVENT.is_set():
        if msvcrt is not None and msvcrt.kbhit():
            handle_key(msvcrt.getwch())

        now = time.perf_counter()
        if now >= next_control:
            update_control()
            next_control = now + control_period
        if now >= next_draw:
            try:
                previous_lines = draw_dashboard(previous_lines)
            except Exception as exc:
                with STATE_LOCK:
                    STATE.conn.muse_status = f"Dashboard error: {type(exc).__name__}"
            next_draw = now + draw_period
        time.sleep(0.005)


# ---------------------------------------------------------------------------
# Startup, shutdown, and self-check
# ---------------------------------------------------------------------------

def signal_handler(sig, frame) -> None:
    STOP_EVENT.set()
    force_stop("signal", immediate=True)


def run_self_check() -> int:
    fake_eeg = bytes(range(28))
    fake_imu = bytes(range(36))
    eeg = decode_eeg_4ch(fake_eeg)
    imu = decode_accgyro(fake_imu)

    assert eeg.shape == (4, 4), eeg.shape
    assert imu.shape == (3, 6), imu.shape
    assert parse_payload(b"short") == []
    assert angle_delta_deg(300.0, 0.0) == -60.0
    assert angle_delta_deg(30.0, 0.0) == 30.0

    detector = BlinkDetector(LEFT_RISE_THRESH, LEFT_MIN_PEAK)
    assert detector.process(np.array([1.0, 2.0, 3.0], dtype=np.float32)) is False

    jaw = JawDetector()
    jaw_arr = np.array(
        [
            [0.0, 0.0, 0.0, 1450.0],
            [55.0, 0.0, 0.0, 1450.0],
            [0.0, 0.0, 0.0, 1450.0],
            [55.0, 0.0, 0.0, 1450.0],
        ],
        dtype=np.float32,
    )
    fired = False
    for i in range(4):
        fired = jaw.process(jaw_arr, calibrating=False, now=200.0 + i)
    assert fired is True
    assert jaw.tp9_over is True
    assert jaw.tp10_over is False
    assert jaw.process(jaw_arr, calibrating=False, now=210.0) is False
    assert jaw.jaw_armed is False
    rest_arr = np.zeros((4, 4), dtype=np.float32)
    for i in range(25):
        jaw.process(rest_arr, calibrating=False, now=211.0 + i)
    assert jaw.jaw_armed is True

    with STATE_LOCK:
        STATE.blink = BlinkBurst()
    register_blink(100.00)
    register_blink(100.40)
    forward, stop = resolve_blinks(100.40)
    assert (forward, stop) == (False, True)
    register_blink(100.60)
    forward, stop = resolve_blinks(101.70)
    assert (forward, stop) == (False, False)

    with CAR_LOCK:
        CAR_CMD.desired = "S"
        CAR_CMD.dirty = False
    force_stop("self-check")
    with CAR_LOCK:
        assert CAR_CMD.dirty is False
    force_stop("self-check immediate", immediate=True)
    with CAR_LOCK:
        assert CAR_CMD.dirty is True

    print("Self-check OK: decoders, parser, roll wrap, jaw edge-trigger, blink safety, STOP heartbeat.")
    if bleak is None:
        print("Note: bleak is not installed in this Python, so hardware run will fail until installed.")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Muse Athena EEG car controller")
    parser.add_argument("--check", action="store_true", help="run safe checks and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.check:
        return run_self_check()

    enable_windows_terminal_colors()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    car = threading.Thread(target=car_thread, name="hc08-car-link", daemon=True)
    ui = threading.Thread(target=ui_thread, name="dashboard", daemon=True)
    car.start()
    ui.start()

    try:
        asyncio.run(muse_main())
    except KeyboardInterrupt:
        STOP_EVENT.set()
    finally:
        force_stop("shutdown", immediate=True)
        STOP_EVENT.set()
        time.sleep(0.3)
        print("\nStopped safely.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
