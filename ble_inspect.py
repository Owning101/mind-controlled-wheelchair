#!/usr/bin/env python3
"""Connects to a BLE device and lists all services and characteristics."""
import asyncio, bleak, sys

TARGETS = [
    ("6F:C4:C6:3A:F2:4C", "(no name)"),
    ("C0:84:FF:BF:D8:83", "midea"),
]

async def inspect(addr, label):
    print(f"\n── {label}  {addr} ──")
    try:
        async with bleak.BleakClient(addr, timeout=8.0) as c:
            for svc in c.services:
                print(f"  SVC  {svc.uuid}  {svc.description}")
                for ch in svc.characteristics:
                    print(f"    CHR  {ch.uuid}  props={ch.properties}")
    except Exception as e:
        print(f"  FAILED: {e}")

async def main():
    for addr, label in TARGETS:
        await inspect(addr, label)

asyncio.run(main())
