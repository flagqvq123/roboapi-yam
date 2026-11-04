#!/usr/bin/env python3
"""Identify installed DM motor types by reading the Gr gear-ratio register (read-only).

No motor is enabled: this only sends DM register-read frames (command 0x33) to the broadcast id,
the same read i2rt startup checks use to refuse a chain whose declared types do not match.
"""
import sys

from i2rt.motor_config_tool.dm_motor_registers import _open_bus, read_register
from i2rt.motor_drivers.utils import MotorType


def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else "can_follower_r"
    ids = [int(v) for v in sys.argv[2].split(",")] if len(sys.argv) > 2 else list(range(1, 9))
    known = MotorType.known_gear_ratios()
    print("channel:", channel, "ids:", ids)
    found = {}
    with _open_bus(channel) as iface:
        for motor_id in ids:
            try:
                gr = read_register(iface, motor_id, "Gr")
            except Exception as exc:
                print(f"id {motor_id}: no answer ({type(exc).__name__}: {exc})")
                continue
            guess = [name for name, ratio in known.items() if abs(float(gr) - ratio) < 0.05]
            found[motor_id] = guess
            print(f"id {motor_id}: Gr={gr} -> {guess or unknown_text()}")
    print()
    print("gripper (id 7):", found.get(7))
    print("known ratios:", known)


def unknown_text():
    return "unknown motor type"


if __name__ == "__main__":
    main()
