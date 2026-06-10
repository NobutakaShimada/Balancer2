#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balancer2_hid import Balancer2HID


def main():
    devices = Balancer2HID.enumerate()
    if not devices:
        print("Balancer2 HID device not found.")
        return

    for index, device in enumerate(devices, start=1):
        print(f"[{index}]")
        for key in sorted(device):
            print(f"  {key}: {device[key]}")


if __name__ == "__main__":
    main()
