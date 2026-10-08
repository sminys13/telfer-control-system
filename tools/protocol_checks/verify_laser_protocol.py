#!/usr/bin/env python3
"""Offline sanity check for the SEN0366-compatible laser command table."""
from __future__ import annotations

COMMANDS = {
    "shutdown": bytes.fromhex("80 04 02 7A"),
    "laser_on": bytes.fromhex("80 06 05 01 74"),
    "range_10m": bytes.fromhex("FA 04 09 0A EF"),
    "resolution_1mm": bytes.fromhex("FA 04 0C 01 F5"),
    "resolution_0_1mm": bytes.fromhex("FA 04 0C 02 F4"),
    "frequency_5hz": bytes.fromhex("FA 04 0A 05 F3"),
    "frequency_10hz": bytes.fromhex("FA 04 0A 0A EE"),
    "frequency_20hz": bytes.fromhex("FA 04 0A 14 E4"),
    "continuous": bytes.fromhex("80 06 03 77"),
    "single": bytes.fromhex("80 06 02 78"),
}

ACKS = {
    "shutdown": bytes.fromhex("80 04 82 FA"),
    "laser_on": bytes.fromhex("80 06 85 01 F4"),
    "range": bytes.fromhex("FA 04 89 79"),
    "resolution": bytes.fromhex("FA 04 8C 76"),
    "frequency": bytes.fromhex("FA 04 8A 78"),
}


def checksum(frame_without_checksum: bytes) -> int:
    return (-sum(frame_without_checksum)) & 0xFF


def verify(name: str, frame: bytes) -> None:
    actual = checksum(frame[:-1])
    if actual != frame[-1]:
        raise SystemExit(f"{name}: checksum mismatch, calculated 0x{actual:02X}, frame 0x{frame[-1]:02X}")
    print(f"OK {name:18s} {frame.hex(' ').upper()}")


def main() -> None:
    for name, frame in {**COMMANDS, **{f"ack_{k}": v for k, v in ACKS.items()}}.items():
        verify(name, frame)
    print("All protocol constants passed two's-complement checksum verification.")


if __name__ == "__main__":
    main()
