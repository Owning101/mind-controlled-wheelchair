"""
car_keyboard_test.py — drive the Arduino car from the keyboard (no Muse needed).

Connects to the HC-08 BLE module (same address/UUID as the BCI controller in
config.py) and sends single-byte motion commands when you press keys.

Controls (press a key to set that motion — it keeps going until you press
another key or STOP):
    W : forward            S : backward
    A : turn left          D : turn right     (curved, no spin)
        — while going forward  -> forward-left / forward-right
        — while going backward -> back-left / back-right
  SPACE / X : stop
    1 / 2 / 3 : speed  (slow / med / fast)
  ESC / Ctrl-C : quit

Run:  python car_keyboard_test.py
"""

import asyncio
import sys
import msvcrt          # Windows-only: non-blocking keyboard reads
import bleak

from config import HC08_ADDRESS, UART_CHAR_UUID

# Simple keys → command byte (turns are handled separately, see resolve()).
KEYMAP = {
    'w': 'F', 's': 'B',
    ' ': 'S', 'x': 'S',
    '1': '1', '2': '2', '3': '3',
}

LABELS = {
    'F': 'FORWARD', 'B': 'BACKWARD',
    'L': 'TURN LEFT', 'R': 'TURN RIGHT',
    'G': 'BACK-LEFT', 'H': 'BACK-RIGHT',
    'S': 'STOP',
    '1': 'SPEED SLOW', '2': 'SPEED MED', '3': 'SPEED FAST',
}


def read_key():
    """Return a lowercased key char if one is waiting, else None. ESC -> 'esc'."""
    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b'\x00', b'\xe0'):   # arrow / function key prefix — consume the next byte
        msvcrt.getch()
        return None
    if ch == b'\x1b':              # ESC
        return 'esc'
    try:
        return ch.decode('ascii').lower()
    except UnicodeDecodeError:
        return None


async def main():
    print(f"Connecting to car {HC08_ADDRESS} ...")
    try:
        async with bleak.BleakClient(HC08_ADDRESS, timeout=10.0) as client:
            print("Connected! WASD to drive, SPACE=stop, ESC=quit.\n")
            await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)

            heading = 'F'   # last drive direction: 'F' forward, 'B' backward
            while True:
                key = read_key()
                if key == 'esc':
                    break

                cmd = None
                if key in ('w', 's'):
                    heading = KEYMAP[key]          # remember forward/backward
                    cmd = heading
                elif key == 'a':                   # turn left, respecting heading
                    cmd = 'G' if heading == 'B' else 'L'
                elif key == 'd':                   # turn right, respecting heading
                    cmd = 'H' if heading == 'B' else 'R'
                elif key in KEYMAP:                # stop / speed
                    cmd = KEYMAP[key]

                if cmd is not None:
                    await client.write_gatt_char(
                        UART_CHAR_UUID, cmd.encode(), response=True)
                    print(f"  {LABELS.get(cmd, cmd):<16} ({cmd})")
                await asyncio.sleep(0.02)

            # stop before disconnecting
            await client.write_gatt_char(UART_CHAR_UUID, b'S', response=True)
            print("\nStopped. Bye.")
    except Exception as e:
        print(f"Could not connect/drive: {type(e).__name__}: {e}")
        sys.exit(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
