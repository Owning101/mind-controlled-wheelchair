# Shared hardware + tuning configuration
# Edit this file to change addresses/thresholds without touching controller code.

# ── HC-08 BLE module ──────────────────────────────────────────────────────────
HC08_ADDRESS   = "A8:E2:C1:63:25:38"
UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# ── IMU thresholds (degrees) ──────────────────────────────────────────────────
ROLL_THRESHOLD       = 15.0   # tilt ear to shoulder → L/R (fallback if no EOG)
PITCH_FWD_THRESHOLD  = 7.0    # nod forward          → Forward
PITCH_BACK_THRESHOLD = 7.0    # lean back            → Backward

# ── EOG thresholds (µV) ───────────────────────────────────────────────────────
EOG_H_THRESHOLD = 30.0   # horizontal gaze asymmetry (AF7 − AF8) → L/R steer

# ── Loop rates ────────────────────────────────────────────────────────────────
LOOP_HZ    = 20
DISPLAY_HZ = 4
