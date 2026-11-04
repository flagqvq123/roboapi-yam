#!/usr/bin/env python3
"""Switch the running YAM adapter between gravity / hold / servo without restarting.

Usage: python scripts/yam_mode.py gravity|hold|servo|status
"""
import os
import sys
import time
from pathlib import Path

MODE_FILE = Path(os.environ.get("ROBOAPI_YAM_MODE_FILE", "~/roboapi/state/yam_mode")).expanduser()


def main():
    args = sys.argv[1:]
    if not args:
        print("usage: yam_mode.py gravity|hold|servo|status")
        return
    value = args[0].strip().lower()
    if value == "status":
        try:
            print("mode file:", MODE_FILE, "content:", MODE_FILE.read_text().strip())
        except FileNotFoundError:
            print("mode file not set:", MODE_FILE)
        return
    if value not in ("gravity", "hold", "servo"):
        print("unknown mode:", value)
        return
    MODE_FILE.parent.mkdir(parents=True, exist_ok=True)
    MODE_FILE.write_text(value + chr(10), encoding="utf-8")
    print("requested mode:", value, "(", MODE_FILE, ")")
    time.sleep(0.5)


if __name__ == "__main__":
    main()
