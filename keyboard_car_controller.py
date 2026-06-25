#!/usr/bin/env python3
"""
Keyboard Car Controller — WASD + Arrow Keys → HC-08 BLE → Arduino
Run: C:\\Users\\Admin\\OneDrive\\Desktop\\eeg_env\\Scripts\\python.exe keyboard_car_controller.py
Stop: Ctrl+C or ESC
"""
import sys, time, ctypes, asyncio
import bleak

# ── Config ─────────────────────────────────────────────────────────────────────
HC08_ADDRESS   = "A8:E2:C1:63:25:38"
UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
POLL_HZ        = 20

# ── ANSI (Windows) ─────────────────────────────────────────────────────────────
_k32 = ctypes.windll.kernel32
_k32.SetConsoleMode(_k32.GetStdHandle(-11),
                    ctypes.c_ulong(7))   # ENABLE_PROCESSED + WRAP + VT

# ── Key polling via GetAsyncKeyState ───────────────────────────────────────────
_u32 = ctypes.windll.user32
def held(vk): return bool(_u32.GetAsyncKeyState(vk) & 0x8000)

VK_W, VK_A, VK_S, VK_D = 0x57, 0x41, 0x53, 0x44
VK_UP, VK_LEFT, VK_DOWN, VK_RIGHT = 0x26, 0x25, 0x28, 0x27
VK_SPACE, VK_ESC = 0x20, 0x1B

def get_command():
    fwd  = held(VK_W) or held(VK_UP)
    back = held(VK_S) or held(VK_DOWN)
    lft  = held(VK_A) or held(VK_LEFT)
    rgt  = held(VK_D) or held(VK_RIGHT)
    stp  = held(VK_SPACE)

    if stp:               return 'S'
    if back:              return 'B'
    if fwd and lft:       return 'Q'   # curve left  — right side drives
    if fwd and rgt:       return 'E'   # curve right — left side drives
    if fwd:               return 'F'
    if lft:               return 'L'   # turn left (curved, no spin)
    if rgt:               return 'R'   # turn right (curved, no spin)
    return 'S'

# ── Display ────────────────────────────────────────────────────────────────────
CMD_LABEL = {
    'F': '\033[92mFORWARD        F\033[0m',
    'B': '\033[93mBACKWARD       B\033[0m',
    'L': '\033[94mTURN LEFT      L\033[0m',
    'R': '\033[94mTURN RIGHT     R\033[0m',
    'Q': '\033[96mCURVE LEFT     Q\033[0m',
    'E': '\033[96mCURVE RIGHT    E\033[0m',
    'S': '\033[91mSTOP           S\033[0m',
}

def print_display(cmd, ble_status):
    fwd  = held(VK_W) or held(VK_UP)
    back = held(VK_S) or held(VK_DOWN)
    lft  = held(VK_A) or held(VK_LEFT)
    rgt  = held(VK_D) or held(VK_RIGHT)

    def khi(active, label):
        return f'\033[97m\033[1m{label}\033[0m' if active else f'\033[90m{label}\033[0m'

    sys.stdout.write('\033[2J\033[H')
    print('  ┌─ KEYBOARD CAR CONTROLLER ────────────────────┐')
    print(f'  │  BLE  : {ble_status:<38}│')
    print(f'  │  CMD  : {CMD_LABEL[cmd]:<47}│')
    print('  │                                              │')
    print(f'  │         {khi(fwd,  "[W / ↑]  FORWARD"):<46}│')
    print(f'  │  {khi(lft, "[A / ←]  LEFT"):<20}  {khi(rgt, "[D / →]  RIGHT"):<20}      │')
    print(f'  │         {khi(back, "[S / ↓]  BACKWARD"):<46}│')
    print('  │                                              │')
    print('  │  SPACE = force stop   ESC / Ctrl+C = quit   │')
    print('  └──────────────────────────────────────────────┘')
    sys.stdout.flush()

# ── BLE main loop ──────────────────────────────────────────────────────────────
async def run():
    last_cmd   = None
    interval   = 1.0 / POLL_HZ

    while True:
        ble_status = '\033[91mConnecting...\033[0m'
        print_display('S', ble_status)

        try:
            async with bleak.BleakClient(HC08_ADDRESS, timeout=10.0) as client:
                ble_status = '\033[92mConnected  ' + HC08_ADDRESS + '\033[0m'

                async def send(cmd):
                    await client.write_gatt_char(
                        UART_CHAR_UUID, cmd.encode(), response=True)

                await send('S')
                last_cmd = 'S'

                while client.is_connected:
                    if held(VK_ESC):
                        await send('S')
                        return

                    cmd = get_command()
                    if cmd != last_cmd:
                        await send(cmd)
                        last_cmd = cmd

                    print_display(cmd, ble_status)
                    await asyncio.sleep(interval)

        except asyncio.CancelledError:
            return
        except Exception:
            pass

        # Disconnected — wait before retry
        for _ in range(30):          # 3 s
            if held(VK_ESC):
                return
            print_display('S', '\033[91mDisconnected — retrying...\033[0m')
            await asyncio.sleep(0.1)

async def main():
    try:
        await run()
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\033[2J\033[H')
        print('  Stopped.')

if __name__ == '__main__':
    asyncio.run(main())
