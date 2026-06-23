#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lpc13xx_iflash import LPC13xxIFlash


def hexdump(data, base=0):
    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{base + offset:08x}: {hex_part:<47}  {ascii_part}")


def main():
    parser = argparse.ArgumentParser(description="Read sectors from NXP LPC13xx IFLASH.")
    parser.add_argument("--lba", type=int, default=0)
    parser.add_argument("--blocks", type=int, default=1)
    parser.add_argument("--limit", type=int, default=512, help="maximum bytes to print")
    args = parser.parse_args()

    with LPC13xxIFlash() as iflash:
        capacity = iflash.read_capacity10()
        block_size = capacity["block_size"]
        data = iflash.read10(args.lba, args.blocks, block_size)
        print("READ(10):")
        print("  lba:", args.lba)
        print("  blocks:", args.blocks)
        print("  block_size:", block_size)
        print("  bytes:", len(data))
        print()
        hexdump(data[: args.limit], base=args.lba * block_size)


if __name__ == "__main__":
    raise SystemExit(main())
