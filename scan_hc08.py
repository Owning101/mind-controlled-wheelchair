#!/usr/bin/env python3
"""
BLE scanner — finds HC-08 and prints its address + service UUIDs.
Run this with the car powered on.
"""
import asyncio
import bleak

async def scan():
    print("Scanning for BLE devices (5 seconds)...\n")
    devices = await bleak.BleakScanner.discover(timeout=5.0)

    if not devices:
        print("No BLE devices found. Is the car powered on?")
        return

    print(f"Found {len(devices)} device(s):\n")
    hc08 = None
    for d in devices:
        name = d.name or "(no name)"
        print(f"  {name:<20}  {d.address}")
        if "HC" in name.upper() or "BT" in name.upper() or "MLT" in name.upper():
            hc08 = d
            print(f"               ^^^ likely your module ^^^")

    if hc08 is None:
        print("\nNo HC-08 spotted by name — check the list above manually.")
        print("Look for anything unfamiliar when the car is ON vs OFF.")
        return

    print(f"\n--- Connecting to {hc08.name} ({hc08.address}) to read services ---")
    async with bleak.BleakClient(hc08.address) as client:
        print(f"Connected: {client.is_connected}\n")
        print("Services and characteristics:")
        for service in client.services:
            print(f"\n  Service  {service.uuid}")
            for char in service.characteristics:
                props = ",".join(char.properties)
                print(f"    Char   {char.uuid}  [{props}]")

    print("\n--- Copy the UUID that has 'write' or 'write-without-response' ---")
    print("--- That is the one we send F/L/R/S to ---")

asyncio.run(scan())
