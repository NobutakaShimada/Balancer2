import time
import functools
#import latency_decorator as ld
from LatencyMeasurer import LatencyMeasurer as ld
from BeuatoMemMap import memory_map

def make_command(var_name: str, com: str) -> bytes:
    """
    var_name に対応するアドレスとバイト長を
    "r addr, len " の形式で組み、ASCII/binary バイト列で返す。
    例: var_name="BODY_ANGLE", com="r" → b"r 18, 8 "
    """
    try:
        addr, length, vartype = memory_map[var_name]
    except KeyError:
        raise ValueError(f"Unknown variable: {var_name!r}")
    # addr を 16 進文字列に（0xなし、小文字）
    addr_hex = format(addr, 'x')
    cmd = f"{com} {addr_hex} {length} "
    return cmd.encode('ascii'), length, vartype

import struct
from typing import Tuple, Union

def parse_beuato_ascii(line: Union[str, bytes]) -> Tuple[bytes, int]:
    """
    Convert an ASCII response of the form
        b"r <len> <b0> <b1> … <bN>\\n"
    into   (payload_bytes, datasize)

    Example
    -------
    >>> line = b"r 2 aa bb \\n"
    >>> payload, n = parse_beuato_ascii(line)
    >>> payload            # b'\\xaa\\xbb'
    >>> struct.unpack('<H', payload)[0]
    48042
    """
    if isinstance(line, bytes):
        line = line.decode('ascii', errors='ignore')

    parts = line.strip().split()
    if len(parts) < 3 or parts[0] != 'r':
        raise ValueError("Malformed Beuato reply")

    datasize = int(parts[1], 16)
    if datasize != len(parts) - 2:
        raise ValueError(f"Length mismatch: header {datasize}, data {len(parts)-2}")

    payload = bytes(int(tok, 16) for tok in parts[2:])
    return payload, datasize

Debug = True


@ld
def send_and_receive(addr, comstr):
    command, length, vartype = make_command(addr, comstr)
    if Debug: print(f'{command}: ({length})')

    # 計測開始
    start_send = time.time()
    dev.write(command)
    #dev.flush()
    end_send = time.time()
    send_time = (end_send - start_send) * 1000 # msec


    #res = dev.readline()
    res = dev.read(64)
    if Debug: print(f'readline: {res}')

    payload, size = parse_beuato_ascii(res)
    if Debug: print(f"size:{size}")

    val = None
    if vartype == "d":
        if size == 8:
            val = struct.unpack("<d", payload)[0]
            if Debug: print(f"double: {val}")
        else:
            print(f"reply len miss match!: {vartype} but size byte received.")
    elif vartype  == "s":
        if size == 2:
            val = struct.unpack("<h", payload)[0]
            if Debug: print(f"short: {val}")
        else:
            print(f"reply len miss match!: {vartype} but size byte received.")
    elif vartype  == "us":
        if size == 2:
            val = struct.unpack("<H", payload)[0]
            if Debug: print(f"ushort: {val}")
        else:
            print(f"reply len miss match!: {vartype} but size byte received.")
    elif vartype  == "c":
        if size == 1:
            val = struct.unpack("<c", payload)[0]
            if Debug: print(f"char: {val}")
        else:
            print(f"reply len miss match!: {vartype} but size byte received.")

    elif vartype  == "uc":
        if size == 1:
            val = struct.unpack("<B", payload)[0]
            if Debug: print(f"uchar: {val}")
        else:
            print(f"reply len miss match!: {vartype} but size byte received.")
    elif vartype  == "ull":
        if size == 8:
            val = struct.unpack("<Q", payload)[0]
            if Debug: print(f"ulonglong: {val}")
        else:
            print(f"reply len miss match!: {vartype} but size byte received.")

    return val


if __name__ == '__main__':
    DRIVER_DEBUG = 0 # 0: no debug 1: debug
    with open('/dev/BeuatoCtrl0', mode='r+b', buffering=0) as dev:
        import fcntl
        fcntl.ioctl(dev, 0x40044200, struct.pack("I",DRIVER_DEBUG)) # debug dmesg

        while True:
            try:
                print("----------------------------")

                #dev.write(b"r 68 2 ")
                #command, length = make_command("BODY_ANGLE","r")
                #length = 63
                #command_str = f"r 0 {length:x} "
                #command = command_str.encode("ascii")

                #import pdb; pdb.set_trace()

                body_angle,timing1 = send_and_receive("BODY_ANGLE", "r")
                print(f"BODY_ANGLE: {body_angle}")

                body_angular_spd, timing2 = send_and_receive("BODY_ANGULAR_SPD", "r")
                print(f"BODY_ANGULAR_SPD: {body_angular_spd}")

                wheel_angle_l, timing3 = send_and_receive("WHEEL_ANGLE_L", "r")
                print(f"WHEEL_ANGLE_L: {wheel_angle_l}")

                gyro_data, timing4  = send_and_receive("GYRO_DATA", "r")
                print(f"GYRO_DATA: {gyro_data}")

                """
                if (i+1)%10 == 0:
                    print(f"  {i+1}/{iterations} 完了")
                    print(f"  最新レイテンシ - BODY_ANGLE: {timing1['latency_ms']:.3f}ms")
                """
            except ValueError as e:
                print(e)
                continue

            except KeyboardInterrupt:
                print("\n計測を中断しました")
                ld.print_stats()
                break
