"""
muse_athena_gatt_dump.py

Diagnostic: scan for any Muse headset, connect, and print every GATT
service + characteristic it exposes (with properties).

Use this when the car controller fails with BleakCharacteristicNotFoundError
after switching headsets — it shows the real UUIDs the new unit uses so we
can update CTRL_UUID / SENSOR_UUID if they differ.

Run:  eeg_env\\Scripts\\python.exe muse_athena_gatt_dump.py
"""
import asyncio
from bleak import BleakScanner, BleakClient

CTRL_UUID   = "273e0001-4c4d-454d-96be-f03bac821358"
SENSOR_UUID = "273e0013-4c4d-454d-96be-f03bac821358"


async def main():
    print("Scanning for any Muse headset (12s)...")
    found = await BleakScanner.discover(timeout=12.0)
    muses = [d for d in found if (d.name or "").lower().startswith("muse")]

    if not muses:
        print("No Muse found. Is the headset on and not connected elsewhere?")
        print("All devices seen:")
        for d in found:
            print(f"  {d.address}  {d.name!r}")
        return

    for d in muses:
        print(f"Found Muse: {d.name}  ({d.address})")

    device = muses[0]
    print(f"\nConnecting to {device.name} ({device.address})...")
    async with BleakClient(device, timeout=20.0) as client:
        print("Connected. GATT table:\n")
        for service in client.services:
            print(f"[service] {service.uuid}  ({service.description})")
            for ch in service.characteristics:
                props = ",".join(ch.properties)
                print(f"    [char] {ch.uuid}  ({props})")
        print()

        all_uuids = {
            ch.uuid.lower()
            for service in client.services
            for ch in service.characteristics
        }
        print("── Controller UUID check ─────────────────────────")
        print(f"CTRL_UUID   {CTRL_UUID}  -> "
              f"{'PRESENT' if CTRL_UUID.lower() in all_uuids else 'MISSING'}")
        print(f"SENSOR_UUID {SENSOR_UUID}  -> "
              f"{'PRESENT' if SENSOR_UUID.lower() in all_uuids else 'MISSING'}")


if __name__ == "__main__":
    asyncio.run(main())
