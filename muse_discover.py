#!/usr/bin/env python3
"""
muse_discover.py
Scans for the Muse S Athena and dumps all its GATT services and characteristics.
Run this once — paste the output so we can identify the correct UUIDs.
"""

import asyncio
from bleak import BleakScanner, BleakClient


async def main():
    print("Scanning for Muse S Athena (up to 10 s)...")
    devices = await BleakScanner.discover(timeout=10.0)

    muse = None
    for d in devices:
        if d.name and ("Muse" in d.name or "muse" in d.name):
            muse = d
            print(f"Found: {d.name}  ({d.address})\n")
            break

    if muse is None:
        print("No Muse device found. Make sure the headset is on.")
        return

    async with BleakClient(muse.address, timeout=15.0) as client:
        print(f"Connected to {muse.name}\n")
        print("=" * 70)

        for service in client.services:
            print(f"\nSERVICE  {service.uuid}")
            print(f"         {service.description}")

            for char in service.characteristics:
                props = ", ".join(char.properties)
                print(f"  CHAR   {char.uuid}  [{props}]")
                print(f"         handle={char.handle}  {char.description}")

                # Try to read readable characteristics for extra context
                if "read" in char.properties:
                    try:
                        val = await client.read_gatt_char(char.uuid)
                        printable = val.decode("ascii", errors="replace").strip()
                        print(f"         value (hex): {val.hex()}")
                        if any(32 <= b < 127 for b in val):
                            print(f"         value (str): {printable!r}")
                    except Exception:
                        pass

        print("\n" + "=" * 70)
        print("Done — paste this output to identify the correct EEG UUIDs.")


if __name__ == "__main__":
    asyncio.run(main())
