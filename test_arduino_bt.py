#!/usr/bin/env python3
"""
BLE keyboard tester for the Arduino wheelchair car (HC-08).

  W / ↑      Forward       S / ↓    Backward
  A / ←      Turn Left     D / →    Turn Right
  Q           Curve Left    E        Curve Right
  SPACE       Stop          X        Quit
"""
import asyncio
import sys
import msvcrt
import bleak

# ── HC-08 BLE config ─────────────────────────────────────────────────────────
HC08_ADDRESS   = "A8:E2:C1:63:25:38"
UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

DIR_LABELS = {
    'F': ('▲', 'FORWARD     ', '\033[92m'),
    'B': ('▼', 'BACKWARD    ', '\033[93m'),
    'L': ('◄', 'TURN LEFT   ', '\033[96m'),
    'R': ('►', 'TURN RIGHT  ', '\033[96m'),
    'Q': ('↖', 'CURVE LEFT  ', '\033[96m'),
    'E': ('↗', 'CURVE RIGHT ', '\033[96m'),
    'S': ('■', 'STOP        ', '\033[90m'),
}

WASD_MAP = {
    b'w': 'F', b'W': 'F',
    b's': 'B', b'S': 'B',
    b'a': 'L', b'A': 'L',
    b'd': 'R', b'D': 'R',
    b'q': 'Q', b'Q': 'Q',
    b'e': 'E', b'E': 'E',
    b' ': 'S',
}
ARROW_MAP = {b'H': 'F', b'K': 'L', b'M': 'R', b'P': 'B'}  # ↑←→↓


def render(direction):
    sym, label, col = DIR_LABELS.get(direction, ('?', '?', ''))
    sys.stdout.write(f'\r  {col}[{direction}] {sym}  {label}\033[0m   ')
    sys.stdout.flush()


async def drive(client, stop_event):
    print("  ┌─ Controls ──────────────────────────────────────────────────┐")
    print("  │  W / ↑   Forward      S / ↓   Backward                     │")
    print("  │  A / ←   Turn Left    D / →   Turn Right                   │")
    print("  │  Q       Curve Left   E       Curve Right                  │")
    print("  │  SPACE   Stop         X       Quit                         │")
    print("  └─────────────────────────────────────────────────────────────┘\n")
    print("  Command → ", end='', flush=True)

    expect_arrow = False

    async def send(cmd):
        try:
            await client.write_gatt_char(UART_CHAR_UUID, cmd.encode(), response=False)
            render(cmd)
        except Exception as e:
            print(f'\n  Send error: {e}')
            stop_event.set()

    while not stop_event.is_set():
        if msvcrt.kbhit():
            key = msvcrt.getch()

            if key in (b'\xe0', b'\x00'):
                expect_arrow = True
                continue

            if expect_arrow:
                expect_arrow = False
                cmd = ARROW_MAP.get(key)
                if cmd:
                    await send(cmd)
                continue

            if key in (b'x', b'X'):
                try:
                    await client.write_gatt_char(UART_CHAR_UUID, b'S', response=False)
                except Exception:
                    pass
                print('\n\n  Stopped.  Goodbye.')
                stop_event.set()
                return

            cmd = WASD_MAP.get(key)
            if cmd:
                await send(cmd)

        await asyncio.sleep(0.01)

    print('\n\n  Disconnected.')


async def main():
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(
        lambda lp, ctx: None if isinstance(ctx.get('exception'), SystemError)
        else lp.default_exception_handler(ctx)
    )

    print('\033[2J\033[H')
    print("  ┌─ Arduino BLE Keyboard Tester (HC-08) ─────────────────────┐")
    print(f"  │  {HC08_ADDRESS}  ·  {UART_CHAR_UUID[:8]}...           │")
    print("  └───────────────────────────────────────────────────────────┘\n")
    print("  Connecting...", end='', flush=True)

    stop_event = asyncio.Event()

    def on_disconnect(client):
        print('\n  BLE disconnected.')
        stop_event.set()

    try:
        async with bleak.BleakClient(
            HC08_ADDRESS,
            timeout=15.0,
            disconnected_callback=on_disconnect
        ) as client:
            print(" Connected ✓\n")
            await drive(client, stop_event)

    except bleak.exc.BleakDeviceNotFoundError:
        print(f"\n  ERROR: HC-08 not found — is the car powered on?")
    except bleak.exc.BleakError as e:
        print(f"\n  BLE error: {e}")
    except Exception as e:
        print(f"\n  Unexpected error: {e}")

    input("\n  Press Enter to close...")


asyncio.run(main())
