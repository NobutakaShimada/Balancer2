#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balancer2_hid import Balancer2HID


def main():
    with Balancer2HID() as dev:
        print("manufacturer:", dev.dev.get_manufacturer_string())
        print("product:", dev.dev.get_product_string())
        print("serial:", dev.dev.get_serial_number_string())


if __name__ == "__main__":
    main()
