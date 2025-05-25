#!/usr/bin/env python3
import time
import struct
import fcntl
from LatencyMeasurer import LatencyMeasurer as ld
from BeuatoMemMap import memory_map, TYPE_FMT

DEVICE_PATH = "/dev/BeuatoCtrl0"
CHUNK_DATA_SIZE = 62           # ドライバから一度に取れるデータ長
CHUNK_READ_SIZE = 256         # ヘッダ('r'+len)を含めた総バッファ


@ld
def read_all_memory(dev):
    """fd 上のデバイスから 0 〜 total_size バイトをチャンク読み出しして返す"""
    # memory_map の最大アドレス＋長さから全体サイズを計算
    max_addr = max(addr + length for addr, length, _ in memory_map.values())
    total_size = ((max_addr + CHUNK_DATA_SIZE - 1) // CHUNK_DATA_SIZE) * CHUNK_DATA_SIZE

    buf = bytearray(total_size)
    for offset in range(0, total_size, CHUNK_DATA_SIZE):
        # コマンド: 'r {addr} {len} '
        cmd = f"r {offset} {CHUNK_DATA_SIZE} ".encode("ascii")
        dev.write(cmd)

        # 256 バイトまで受け取り
        data = dev.read(CHUNK_READ_SIZE)
        if len(data) < 2 or data[0:1] != b"r":
            raise IOError(f"Invalid header: {data!r}")
        data_len = data[1]
        chunk = data[2:2 + data_len]
        buf[offset:offset + len(chunk)] = chunk

    return buf

@ld
def parse_and_dump(buf, memory_map):
    for name, entry in memory_map.items():
        addr, length, typ = entry
        #print(f'{name}: {addr}, {length}, {typ}')
        raw = buf[addr:addr + length]
        if typ == "c":
            val = raw.rstrip(b"\x00").decode("ascii", errors="ignore")
        else:
            fmt = TYPE_FMT.get(typ)
            if fmt is None:
                raise ValueError(f"Unknown type code: {typ}")
            val = struct.unpack_from(fmt, raw)[0]
        #print(f"{name:20s} = {val}")
    #print("-" * 40)


def main():
    # open with no buffering, binary read/write
    with open(DEVICE_PATH, mode="r+b", buffering=0) as dev:
        # optional debug ioctl
        DRIVER_DEBUG = 0
        fcntl.ioctl(dev, 0x40044200, struct.pack("I", DRIVER_DEBUG))

        try:
            while True:
                buf= read_all_memory(dev)
                parse_and_dump(buf, memory_map)
        except KeyboardInterrupt:
            # 測定結果を取得
            raw = ld.get_raw_data('read_all_memory')
            times = raw.get('read_all_memory', [])
            ld.print_stats()


if __name__ == "__main__":
    main()
