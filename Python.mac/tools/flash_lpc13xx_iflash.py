#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lpc13xx_iflash import LPC13xxIFlash


def pad_to_block(data, block_size):
    remainder = len(data) % block_size
    if remainder == 0:
        return data
    return data + b"\x00" * (block_size - remainder)


def main():
    parser = argparse.ArgumentParser(
        description="Write a raw firmware image to NXP LPC13xx IFLASH using USB MSC BOT."
    )
    parser.add_argument("image", help="raw firmware .bin image")
    parser.add_argument("--lba", type=int, default=4, help="start LBA, default 4 for LPC13xx IFLASH FAT data area")
    parser.add_argument("--chunk-blocks", type=int, default=8, help="WRITE(10) chunk size in blocks")
    parser.add_argument("--yes", action="store_true", help="actually write; without this, only prints the plan")
    parser.add_argument("--no-verify", action="store_true", help="skip read-back verification")
    parser.add_argument(
        "--allow-lba0",
        action="store_true",
        help="dangerous: allow overwriting the FAT boot sector and metadata",
    )
    args = parser.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()

    if not image:
        raise SystemExit("image is empty")

    with LPC13xxIFlash() as iflash:
        inquiry = iflash.inquiry()
        capacity = iflash.read_capacity10()
        block_size = capacity["block_size"]
        padded = pad_to_block(image, block_size)
        blocks = len(padded) // block_size

        print("Target:")
        print("  vendor:", inquiry["vendor"])
        print("  product:", inquiry["product"])
        print("  revision:", inquiry["revision"])
        print("  capacity bytes:", capacity["bytes"])
        print("  block size:", block_size)
        print("Image:")
        print("  path:", args.image)
        print("  bytes:", len(image))
        print("  padded bytes:", len(padded))
        print("  start lba:", args.lba)
        print("  blocks:", blocks)

        if args.lba < 0:
            raise SystemExit("start LBA must be >= 0")
        if args.lba == 0 and not args.allow_lba0:
            raise SystemExit(
                "refusing to write from LBA 0 because that overwrites the FAT boot sector. "
                "Use --lba 4 for the LPC13xx IFLASH data area, or --allow-lba0 only if you "
                "intentionally want to destroy the filesystem metadata."
            )
        if args.lba + blocks > capacity["blocks"]:
            raise SystemExit(
                f"image does not fit: needs LBA {args.lba}..{args.lba + blocks - 1}, "
                f"device last LBA is {capacity['last_lba']}"
            )

        if not args.yes:
            print()
            print("Dry run only. Re-run with --yes to write.")
            return 0

        print()
        print("Writing...")
        offset = 0
        lba = args.lba
        chunk_size = args.chunk_blocks * block_size
        while offset < len(padded):
            chunk = padded[offset : offset + chunk_size]
            iflash.write10(lba, chunk, block_size)
            print(f"  wrote LBA {lba}..{lba + len(chunk) // block_size - 1}")
            lba += len(chunk) // block_size
            offset += len(chunk)

        try:
            iflash.synchronize_cache10()
        except Exception as exc:
            print("SYNCHRONIZE CACHE failed; continuing:", exc)

        if not args.no_verify:
            print("Verifying...")
            read_back = bytearray()
            remaining = blocks
            lba = args.lba
            while remaining > 0:
                count = min(args.chunk_blocks, remaining)
                read_back.extend(iflash.read10(lba, count, block_size))
                lba += count
                remaining -= count
            if bytes(read_back[: len(image)]) != image:
                raise SystemExit("verify failed: read-back data differs from image")
            print("Verify OK.")

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
