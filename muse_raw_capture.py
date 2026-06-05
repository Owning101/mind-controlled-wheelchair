#!/usr/bin/env python3
"""
muse_raw_capture.py — Muse S Athena raw packet capture
Order: connect -> CTRL subscribe -> h -> p21 -> subscribe DATA channels -> d -> listen
"""

import asyncio
import time
from bleak import BleakClient

MUSE_ADDR = "00:55:DA:B9:FC:10"

CTRL  = "273e0001-4c4d-454d-96be-f03bac821358"
DATA1 = "273e0013-4c4d-454d-96be-f03bac821358"
DATA2 = "273e0014-4c4d-454d-96be-f03bac821358"
DATA3 = "273e0015-4c4d-454d-96be-f03bac821358"

def cmd(s: str) -> bytes:
    b = s.encode()
    return bytes([len(b)]) + b

t0    = time.time()
total = 0
log   = []

def handler(name):
    def _h(handle, data):
        global total
        total += 1
        ts      = time.time() - t0
        hex_str = data.hex()
        log.append((ts, name, hex_str))
        print(f"[{ts:6.2f}s] {name[:8]}  {hex_str}")
    return _h

async def send(client, uuid, payload, label):
    try:
        await client.write_gatt_char(uuid, payload, response=False)
        print(f"  sent '{label}'")
    except Exception as e:
        print(f"  ERR  '{label}': {e}")

async def main():
    print(f"Connecting to {MUSE_ADDR}...")
    async with BleakClient(MUSE_ADDR, timeout=20.0) as client:
        print("Connected. Settling 2s...")
        await asyncio.sleep(2.0)

        # Step 1: subscribe to CTRL only first (for command responses)
        print("Step 1 — subscribe CTRL...")
        try:
            await client.start_notify(CTRL, handler(CTRL))
            print("  OK CTRL")
        except Exception as e:
            print(f"  ERR CTRL: {e}")

        await asyncio.sleep(0.5)

        # Step 2: halt + set preset
        print("Step 2 — halt then set preset 21...")
        await send(client, CTRL, cmd("h\n"),   "h (halt)")
        await asyncio.sleep(0.5)
        await send(client, CTRL, cmd("p21\n"), "p21 (EEG+PPG+IMU)")
        await asyncio.sleep(1.0)

        # Step 3: subscribe to data channels AFTER preset is set
        print("Step 3 — subscribe data channels...")
        for name, uuid in [("DATA1", DATA1), ("DATA2", DATA2), ("DATA3", DATA3)]:
            try:
                await client.start_notify(uuid, handler(uuid))
                print(f"  OK {name}")
            except Exception as e:
                print(f"  ERR {name}: {e}")

        await asyncio.sleep(0.5)

        # Step 4: start streaming
        print("Step 4 — start streaming...")
        await send(client, CTRL, cmd("d\n"), "d (start streaming)")
        await asyncio.sleep(0.5)

        # Also try alternative start commands
        for c in ["r\n", "s\n", "b\n"]:
            await send(client, CTRL, cmd(c), c.strip())
            await asyncio.sleep(0.3)

        print(f"\n── Listening for 20s ───────────────────────")
        await asyncio.sleep(20.0)

        # Summary
        print(f"\n{'='*55}")
        print(f"Total packets: {total}")
        channels = {}
        for ts, name, h in log:
            channels.setdefault(name, []).append((ts, h))
        for name, pkts in channels.items():
            print(f"\n  {name[:8]}  ({len(pkts)} packets)")
            for ts, h in pkts[:10]:
                print(f"    [{ts:6.2f}s]  {h}")
            if len(pkts) > 10:
                print(f"    ... ({len(pkts)-10} more)")
        print("="*55)

if __name__ == "__main__":
    asyncio.run(main())
