#!/usr/bin/env python3
"""Tries each write characteristic on the car BLE module and sends F (forward)."""
import asyncio, bleak

ADDR = "C0:84:FF:BF:D8:83"

CANDIDATES = [
    "0000ffa1-0000-1000-8000-00805f9b34fb",
    "0000ff81-0000-1000-8000-00805f9b34fb",
    "0000ff91-0000-1000-8000-00805f9b34fb",
]

async def main():
    print(f"Connecting to {ADDR}...")
    async with bleak.BleakClient(ADDR, timeout=10.0) as client:
        print("Connected.\n")
        for uuid in CANDIDATES:
            print(f"Trying {uuid}  → sending F (forward) ...", end=" ", flush=True)
            try:
                await client.write_gatt_char(uuid, b'F', response=False)
                print("OK — did the car move? (waiting 2 s then sending S)")
                await asyncio.sleep(2)
                await client.write_gatt_char(uuid, b'S', response=False)
                print(f"\n>>> If the car moved, this is the correct UUID:\n    {uuid}\n")
            except Exception as e:
                print(f"FAILED ({type(e).__name__})")
            await asyncio.sleep(1)

asyncio.run(main())
