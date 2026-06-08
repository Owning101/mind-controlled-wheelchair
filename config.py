# Shared hardware + tuning configuration
# Edit this file to change addresses/thresholds without touching controller code.

# ── HC-08 BLE module ──────────────────────────────────────────────────────────
HC08_ADDRESS   = "A8:E2:C1:63:25:38"
UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# ── Wink / eye-steering tuning ───────────────────────────────────────────────
# How close (in seconds) the left and right blink detectors must fire to be
# treated as ONE both-eye blink rather than two independent winks.
WINK_COINCIDENCE = 0.20   # s  max gap between eyes for a normal blink; also the
                          #    wink confirm delay. Raise if blinks split into winks.

# How long a wink holds the curved-turn command before reverting to straight.
WINK_PULSE_DUR   = 0.78   # s  (increase for a wider arc; 0.4–1.0 range) — +30% angle

# ── Drive speed ───────────────────────────────────────────────────────────────
# Speed sent to the Arduino once on connect, so the Muse drives at the SAME
# speed as the keyboard tester. '1' = SLOW, '2' = MED, '3' = FAST.
DEFAULT_SPEED = '2'   # MED (matches Arduino boot default)

# ── Legacy IMU / EOG thresholds ───────────────────────────────────────────────
# The Muse Athena controller no longer uses head-tilt steering, but the older
# sister controllers (muse_athena_car_jaw.py, brainflow_car_controller.py,
# wheelchair_controller.py) still import these. Kept so those keep running.
ROLL_THRESHOLD       = 15.0   # tilt ear to shoulder → L/R
PITCH_FWD_THRESHOLD  = 7.0    # nod forward          → Forward
PITCH_BACK_THRESHOLD = 7.0    # lean back            → Backward
EOG_H_THRESHOLD      = 55.0   # horizontal gaze asymmetry (AF7 − AF8) → L/R steer

# ── Loop rates ────────────────────────────────────────────────────────────────
LOOP_HZ    = 20
DISPLAY_HZ = 4
