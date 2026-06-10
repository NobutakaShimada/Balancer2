#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from beuato_transport import (
    BEUATO_MODE_ASCII,
    BEUATO_MODE_BINARY,
    ascii_command_to_hid_payload,
    open_beuato_device,
)


def dump(label, data):
    data = bytes(data)
    ascii_text = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    print(f"{label} HEX:", data.hex(" "))
    print(f"{label} ASC:", ascii_text)


def main():
    parser = argparse.ArgumentParser(description="Send a BeuatoCtrl-style command to Balancer2 HID.")
    parser.add_argument("command", nargs="?", default="r 0 2 ", help='example: "r 0 2 "')
    parser.add_argument("--ascii", action="store_true", help="return driver-compatible ASCII response")
    parser.add_argument("--timeout-ms", type=int, default=1000)
    args = parser.parse_args()

    command = args.command.encode("ascii")
    dump("CMD", command)
    dump("HID", ascii_command_to_hid_payload(command))

    read_mode = BEUATO_MODE_ASCII if args.ascii else BEUATO_MODE_BINARY
    with open_beuato_device(timeout_ms=args.timeout_ms, read_mode=read_mode, force_hid=True) as dev:
        dev.write(command)
        response = dev.read(256)
        dump("RX", response)


if __name__ == "__main__":
    main()
