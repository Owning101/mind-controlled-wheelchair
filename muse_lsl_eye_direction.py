import os
import sys
import numpy as np
from pylsl import StreamInlet, resolve_streams
from scipy.signal import butter, lfilter
from collections import deque
import time

# Enable ANSI VT100 on Windows via ctypes (more reliable than os.system(''))
if sys.platform == 'win32':
    try:
        import ctypes
        _k32    = ctypes.windll.kernel32
        _handle = _k32.GetStdHandle(-11)
        _mode   = ctypes.c_ulong()
        _k32.GetConsoleMode(_handle, ctypes.byref(_mode))
        _k32.SetConsoleMode(_handle, _mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        pass

# ══════════════════════════════════════════════════════
#  ANSI COLOURS
# ══════════════════════════════════════════════════════
RST  = '\033[0m'
BOLD = '\033[1m'
DIM  = '\033[2m'
CYAN = '\033[96m'    # LEFT / BLINK_LEFT
YLW  = '\033[93m'    # RIGHT / BLINK_RIGHT
GRN  = '\033[92m'    # CENTER
MAG  = '\033[95m'    # UP / BLINK_DOUBLE
RED  = '\033[91m'    # DOWN / JAW_CLENCH
WHT  = '\033[97m'    # BLINK_BOTH
GRY  = '\033[90m'    # NONE / NO SIGNAL

# ══════════════════════════════════════════════════════
#  CONFIG  (tune after calibration)
# ══════════════════════════════════════════════════════
SAMPLE_RATE      = 256
DIR_WINDOW       = int(0.50 * SAMPLE_RATE)   # 128 samples
SPIKE_SAMPLES    = int(0.030 * SAMPLE_RATE)  #   8 samples — spike window  (~30 ms)
BASE_SAMPLES     = int(0.300 * SAMPLE_RATE)  #  77 samples — baseline window (~300 ms)
JAW_WINDOW       = int(0.30 * SAMPLE_RATE)   #  77 samples

DIR_THRESHOLD    = 30     # µV  horizontal gaze asymmetry (AF7 - AF8)
VERT_THRESHOLD   = 25     # µV  vertical gaze sum         (AF7 + AF8)
BLINK_SPIKE_RATIO = 3.0   # spike must be >= this × baseline mean to count
BLINK_ABS_MIN    = 80.0   # µV  absolute floor (prevents firing on near-flat signal)
BLINK_RATIO      = 2.5    # AF7/AF8 ratio to call a single-eye blink
JAW_THRESHOLD    = 200    # µV  RMS of high-freq signal for jaw clench
HOLD_TIME        = 3.0    # s   hold a direction to fire CMD
DOUBLE_BLINK_WIN = 0.60   # s   window to count two blinks as one double
BLINK_DEBOUNCE   = 1.00   # s   min gap between blink detections
JAW_DEBOUNCE     = 2.50   # s   min gap between jaw detections
EVENT_HOLD       = 1.50   # s   show blink/jaw before resetting to NONE
DISPLAY_RATE     = 0.10   # s   dashboard refresh rate

MIN_SIGNAL_STD   = 0.5    # µV  below = flat / not on head
MAX_SIGNAL_STD   = 800.0  # µV  above = wild noise / not on head

# ══════════════════════════════════════════════════════
#  FILTERS
# ══════════════════════════════════════════════════════
def bandpass(data, low, high, fs=SAMPLE_RATE, order=4):
    nyq = 0.5 * fs
    b, a = butter(order, [low / nyq, high / nyq], btype='band')
    return lfilter(b, a, data)

def highpass(data, cutoff=20, fs=SAMPLE_RATE, order=4):
    nyq = 0.5 * fs
    b, a = butter(order, cutoff / nyq, btype='high')
    return lfilter(b, a, data)

# ══════════════════════════════════════════════════════
#  BUFFERS  (Muse 2 channel order: TP9, AF7, AF8, TP10)
# ══════════════════════════════════════════════════════
tp9_buf  = deque(maxlen=JAW_WINDOW)
af7_buf  = deque(maxlen=DIR_WINDOW)
af8_buf  = deque(maxlen=DIR_WINDOW)
tp10_buf = deque(maxlen=JAW_WINDOW)

# ══════════════════════════════════════════════════════
#  STATE
# ══════════════════════════════════════════════════════
dir_state         = None
dir_state_start   = None
dir_triggered     = False
last_blink_time   = 0.0
last_jaw_time     = 0.0
last_display_time = 0.0

blink_display      = "NONE"
blink_display_time = 0.0
jaw_display        = "NONE"
jaw_display_time   = 0.0

last_event = None    # (timestamp, label)

# ══════════════════════════════════════════════════════
#  SIGNAL QUALITY CHECK
# ══════════════════════════════════════════════════════
def headset_worn():
    if len(af7_buf) < DIR_WINDOW:
        return False
    std = float(np.std(np.array(af7_buf)))
    return MIN_SIGNAL_STD < std < MAX_SIGNAL_STD

# ══════════════════════════════════════════════════════
#  DETECTION — EYE DIRECTION  (LEFT / RIGHT / UP / DOWN / CENTER)
# ══════════════════════════════════════════════════════
def detect_direction():
    global dir_state, dir_state_start, dir_triggered

    if len(af7_buf) < DIR_WINDOW:
        return "CENTER", None

    af7_f = bandpass(np.array(af7_buf), 0.5, 10)
    af8_f = bandpass(np.array(af8_buf), 0.5, 10)

    horiz = float(np.mean(af7_f - af8_f))   # positive → LEFT,  negative → RIGHT
    vert  = float(np.mean(af7_f + af8_f))   # positive → UP,    negative → DOWN

    # Whichever axis dominates wins; CENTER if neither clears threshold
    if abs(vert) >= VERT_THRESHOLD and abs(vert) >= abs(horiz):
        live = "UP" if vert > 0 else "DOWN"
    elif horiz > DIR_THRESHOLD:
        live = "LEFT"
    elif horiz < -DIR_THRESHOLD:
        live = "RIGHT"
    else:
        live = "CENTER"

    now = time.time()
    if live != dir_state:
        dir_state       = live
        dir_state_start = now
        dir_triggered   = False

    confirmed = None
    if dir_state not in (None, "CENTER") and not dir_triggered:
        if now - dir_state_start >= HOLD_TIME:
            dir_triggered = True
            confirmed     = f"CMD:{dir_state}"

    return live, confirmed

# ══════════════════════════════════════════════════════
#  DETECTION — BLINKS
# ══════════════════════════════════════════════════════
def detect_blink():
    global last_blink_time

    needed = BASE_SAMPLES + SPIKE_SAMPLES
    if len(af7_buf) < needed:
        return None

    now = time.time()
    if now - last_blink_time < BLINK_DEBOUNCE:
        return None

    af7_arr = np.array(af7_buf)[-needed:]
    af8_arr = np.array(af8_buf)[-needed:]

    # Baseline: mean absolute amplitude of the 300 ms before the spike window
    af7_base  = float(np.mean(np.abs(af7_arr[:-SPIKE_SAMPLES])))
    af8_base  = float(np.mean(np.abs(af8_arr[:-SPIKE_SAMPLES])))

    # Spike: peak absolute amplitude in the last 30 ms
    af7_spike = float(np.max(np.abs(af7_arr[-SPIKE_SAMPLES:])))
    af8_spike = float(np.max(np.abs(af8_arr[-SPIKE_SAMPLES:])))

    # Fire only if the spike is a genuine jump above baseline, not just a high DC level
    af7_fired = af7_spike > BLINK_SPIKE_RATIO * af7_base and af7_spike > BLINK_ABS_MIN
    af8_fired = af8_spike > BLINK_SPIKE_RATIO * af8_base and af8_spike > BLINK_ABS_MIN

    if not (af7_fired or af8_fired):
        return None

    if af7_fired and af8_fired:
        ratio = af7_spike / max(af8_spike, 1e-6)
        if ratio > BLINK_RATIO:
            btype = "BLINK_LEFT"
        elif ratio < 1.0 / BLINK_RATIO:
            btype = "BLINK_RIGHT"
        else:
            btype = "BLINK_BOTH"
    elif af7_fired:
        btype = "BLINK_LEFT"
    else:
        btype = "BLINK_RIGHT"

    if last_blink_time > 0 and now - last_blink_time <= DOUBLE_BLINK_WIN:
        btype = "BLINK_DOUBLE"

    last_blink_time = now
    return btype

# ══════════════════════════════════════════════════════
#  DETECTION — JAW CLENCH
# ══════════════════════════════════════════════════════
def detect_jaw():
    global last_jaw_time

    if len(tp9_buf) < JAW_WINDOW:
        return None

    now = time.time()
    if now - last_jaw_time < JAW_DEBOUNCE:
        return None

    tp9_hf  = highpass(np.array(tp9_buf))
    tp10_hf = highpass(np.array(tp10_buf))
    rms = float(np.sqrt(np.mean((tp9_hf ** 2 + tp10_hf ** 2) / 2.0)))

    if rms > JAW_THRESHOLD:
        last_jaw_time = now
        return "JAW_CLENCH"
    return None

# ══════════════════════════════════════════════════════
#  DISPLAY
# ══════════════════════════════════════════════════════
W1, W2, W3 = 12, 14, 11   # visible content width for each column

DIRECTION_STYLE = {
    "LEFT":      CYAN + BOLD,
    "RIGHT":     YLW  + BOLD,
    "CENTER":    GRN  + BOLD,
    "UP":        MAG  + BOLD,
    "DOWN":      RED  + BOLD,
    "NO SIGNAL": GRY,
}
BLINK_STYLE = {
    "BLINK_LEFT":   CYAN + BOLD,
    "BLINK_RIGHT":  YLW  + BOLD,
    "BLINK_BOTH":   WHT  + BOLD,
    "BLINK_DOUBLE": MAG  + BOLD,
    "NONE":         GRY,
    "NO SIGNAL":    GRY,
}

def styled(label, style_map, fallback=BOLD):
    style = style_map.get(label, fallback)
    return style + label + RST

def styled_jaw(label):
    if label in ("NONE", "NO SIGNAL"):
        return GRY + label + RST
    return RED + BOLD + label + RST

def cpad(raw, colored, width):
    """Centre `colored` (ANSI string) in `width` visible chars using `raw` for length."""
    pad = max(0, width - len(raw))
    return ' ' * (pad // 2) + colored + ' ' * (pad - pad // 2)

def render(direction, blink_disp, jaw_disp, event):
    top = f"╔{'═'*(W1+2)}╦{'═'*(W2+2)}╦{'═'*(W3+2)}╗"
    hdr = f"║{'DIRECTION':^{W1+2}}║{'BLINK':^{W2+2}}║{'JAW':^{W3+2}}║"
    sep = f"╠{'═'*(W1+2)}╬{'═'*(W2+2)}╬{'═'*(W3+2)}╣"
    bot = f"╚{'═'*(W1+2)}╩{'═'*(W2+2)}╩{'═'*(W3+2)}╝"

    d_cell = cpad(direction,  styled(direction,  DIRECTION_STYLE), W1)
    b_cell = cpad(blink_disp, styled(blink_disp, BLINK_STYLE),    W2)
    j_cell = cpad(jaw_disp,   styled_jaw(jaw_disp),               W3)
    row    = f"║ {d_cell} ║ {b_cell} ║ {j_cell} ║"

    if event:
        ts  = time.strftime("%H:%M:%S", time.localtime(event[0]))
        evl = f"  {BOLD}>>>{RST} {event[1]}  {DIM}[{ts}]{RST}"
    else:
        evl = f"  {GRY}(no events yet){RST}"

    hint = f"  {DIM}Hold direction 3 s → CMD  |  Ctrl-C to quit{RST}"

    block = [top, hdr, sep, row, bot, "", evl, hint]

    # \033[2J = clear screen,  \033[H = cursor to top-left
    # This replaces cursor-up which breaks when old output is in the scroll buffer
    sys.stdout.write('\033[2J\033[H' + '\n'.join(block) + '\n')
    sys.stdout.flush()

# ══════════════════════════════════════════════════════
#  CONNECT TO LSL
# ══════════════════════════════════════════════════════
print("Looking for EEG stream...")
streams = resolve_streams(wait_time=5)
eeg_streams = [s for s in streams if s.type() == 'EEG']

if not eeg_streams:
    raise RuntimeError(
        "No EEG stream found.\n"
        "Start streaming in another terminal:\n"
        "  muselsl stream\n"
        "or:  muselsl stream --address XX:XX:XX:XX:XX:XX"
    )

inlet = StreamInlet(eeg_streams[0])
print("Connected to Muse 2 EEG stream — starting display...")
time.sleep(1)   # brief pause so the message is visible before screen clears

# ══════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════
while True:
    sample, _ = inlet.pull_sample()
    tp9_buf.append(sample[0])
    af7_buf.append(sample[1])
    af8_buf.append(sample[2])
    tp10_buf.append(sample[3])

    now  = time.time()
    worn = headset_worn()

    if not worn:
        if now - last_display_time >= DISPLAY_RATE:
            render("NO SIGNAL", "NO SIGNAL", "NO SIGNAL", last_event)
            last_display_time = now
        continue

    live, confirmed = detect_direction()
    blink           = detect_blink()
    jaw             = detect_jaw()

    if blink:
        blink_display      = blink
        blink_display_time = now
        last_event = (now, blink)
    elif now - blink_display_time > EVENT_HOLD:
        blink_display = "NONE"

    if jaw:
        jaw_display      = jaw
        jaw_display_time = now
        last_event = (now, jaw)
    elif now - jaw_display_time > EVENT_HOLD:
        jaw_display = "NONE"

    if confirmed:
        last_event = (now, confirmed)

    if now - last_display_time >= DISPLAY_RATE:
        render(live, blink_display, jaw_display, last_event)
        last_display_time = now
