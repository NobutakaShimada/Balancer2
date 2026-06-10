#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import usb.core
import usb.util

from lpc13xx_iflash import LPC13xxIFlash, find_iflash_device


def hex_bytes(data):
    return bytes(data).hex(" ")


def dump_descriptor(dev):
    print(f"DEVICE {dev.idVendor:04x}:{dev.idProduct:04x}")
    print("manufacturer:", usb.util.get_string(dev, dev.iManufacturer) if dev.iManufacturer else "")
    print("product:", usb.util.get_string(dev, dev.iProduct) if dev.iProduct else "")
    print("serial:", usb.util.get_string(dev, dev.iSerialNumber) if dev.iSerialNumber else "")
    print("configurations:", dev.bNumConfigurations)
    for cfg in dev:
        print(f"CONFIG {cfg.bConfigurationValue}")
        for intf in cfg:
            print(
                "  INTERFACE",
                intf.bInterfaceNumber,
                f"class={intf.bInterfaceClass:#04x}",
                f"subclass={intf.bInterfaceSubClass:#04x}",
                f"protocol={intf.bInterfaceProtocol:#04x}",
            )
            for ep in intf:
                direction = "IN" if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN else "OUT"
                print(
                    "    ENDPOINT",
                    f"0x{ep.bEndpointAddress:02x}",
                    direction,
                    f"attr=0x{ep.bmAttributes:02x}",
                    f"max_packet={ep.wMaxPacketSize}",
                )


def main():
    dev = find_iflash_device()
    if dev is None:
        print("NXP LPC13xx IFLASH not found.")
        return 1

    dump_descriptor(dev)
    print()

    try:
        with LPC13xxIFlash(dev) as iflash:
            print("claimed interface:", iflash.interface.bInterfaceNumber)
            print("bulk out:", f"0x{iflash.ep_out.bEndpointAddress:02x}")
            print("bulk in:", f"0x{iflash.ep_in.bEndpointAddress:02x}")
            print("max lun:", iflash.get_max_lun())

            print()
            try:
                inquiry = iflash.inquiry()
                print("INQUIRY:")
                print("  vendor:", inquiry["vendor"])
                print("  product:", inquiry["product"])
                print("  revision:", inquiry["revision"])
                print("  removable:", inquiry["removable"])
                print("  raw:", hex_bytes(inquiry["raw"]))
            except Exception as exc:
                print("INQUIRY failed:", exc)
                try:
                    print("REQUEST SENSE after INQUIRY:", iflash.request_sense())
                except Exception as sense_exc:
                    print("REQUEST SENSE failed:", sense_exc)
                return 2

            print()
            try:
                print("TEST UNIT READY:", iflash.test_unit_ready())
            except Exception as exc:
                print("TEST UNIT READY failed:", exc)
                try:
                    print("REQUEST SENSE:", iflash.request_sense())
                except Exception as sense_exc:
                    print("REQUEST SENSE failed:", sense_exc)

            print()
            try:
                capacity = iflash.read_capacity10()
                print("READ CAPACITY(10):")
                print("  last_lba:", capacity["last_lba"])
                print("  block_size:", capacity["block_size"])
                print("  blocks:", capacity["blocks"])
                print("  bytes:", capacity["bytes"])
                print("  raw:", hex_bytes(capacity["raw"]))

                first_sector = iflash.read10(0, 1, capacity["block_size"])
                print()
                print("READ(10) LBA 0:")
                print("  bytes:", len(first_sector))
                print("  first 64 bytes:", hex_bytes(first_sector[:64]))
            except Exception as exc:
                print("READ CAPACITY(10) failed:", exc)
                try:
                    print("REQUEST SENSE:", iflash.request_sense())
                except Exception as sense_exc:
                    print("REQUEST SENSE failed:", sense_exc)
    except Exception as exc:
        print("open/claim failed:", exc)
        try:
            print("kernel driver active:", dev.is_kernel_driver_active(0))
        except Exception as active_exc:
            print("kernel driver active check failed:", active_exc)
        try:
            dev.detach_kernel_driver(0)
            print("detach_kernel_driver succeeded")
        except Exception as detach_exc:
            print("detach_kernel_driver failed:", detach_exc)
        return 3

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
