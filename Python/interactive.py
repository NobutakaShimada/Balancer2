import time
import functools
#import latency_decorator as ld
from LatencyMeasurer import LatencyMeasurer as ld
from BeuatoMemMap import memory_map, TYPE_FMT
import ast

# automatic string to numerical
convert = lambda s: ast.literal_eval(s) if s.replace('.','').replace('-','').replace('e','').replace('E','').replace('+','').isdigit() else s


IOCTL_DEBUG = 0x40044200
IOCTL_READ_MODE = 0x40044201

Debug = False

def make_command(var_name: str, com: str) -> bytes:
    """
    var_name に対応するアドレスとバイト長を
    "r addr, len " の形式で組み、ASCII/binary バイト列で返す。
    例: var_name="BODY_ANGLE", com="r" → b"r 18 8 "
    """
    try:
        addr, length, vartype = memory_map[var_name]
    except KeyError:
        raise ValueError(f"Unknown variable: {var_name!r}")
    # addr を 16 進文字列に（0xなし、小文字）
    b_cmd = make_command_from_addr(addr, length, com)
    return b_cmd, length, vartype

def make_write_command(name, data, com: str) -> bytes:
    try:
        addr, length, vartype = memory_map[name]
    except KeyError:
        raise ValueError(f"Unknown variable: {var_name!r}")
    addr_hex = format(addr, 'x')
    len_hex = format(length, 'x')
    bdata = struct.pack(TYPE_FMT[vartype], convert(data))
    cmd = f"{com} {addr_hex} {len_hex} {bdata.hex(' ')}"
    return cmd.encode('ascii')

def make_read_command_from_addr(addr, length, com: str) -> bytes:
    addr_hex = format(addr, 'x')
    len_hex = format(length, 'x')
    cmd = f"{com} {addr_hex} {len_hex} "
    return cmd.encode('ascii')

def make_command_from_str(str) -> bytes:
    cmd = str
    return cmd.encode('ascii')

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
    if Debug:
        print("parts: ", end="")
        for p in parts:
            print(f"{p}", end="/")
        print("")

    if len(parts) < 3 or parts[0] != 'r':
        raise ValueError("Malformed Beuato reply")

    datasize = int(parts[1], 16)
    if datasize != len(parts) - 2:
        raise ValueError(f"Length mismatch: header {datasize}, data {len(parts)-2}")

    payload = bytes(int(tok, 16) for tok in parts[2:])
    #print(f'payload: {payload}')
    return payload, datasize




def send_and_receive_command_only(command):
    dev.write(command)

    res = dev.read(256)
    if Debug: print(f'readline: {res}')

    return res

def responce_decode(payload, size, vartype):
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

def send_and_receive_command(command, length, vartype):
    res = send_and_receive_command_only(command)

    payload, size = parse_beuato_ascii(res)
    if Debug: print(f"payload:{payload} size:{size}")

    return responce_decode(payload, size, vartype)

@ld
def send_and_receive(addr, comstr):
    command, length, vartype = make_command(addr, comstr)
    if Debug: print(f'{command}: ({length})')
    return send_and_receive_command(command, length, vartype)



if __name__ == '__main__':
    DRIVER_DEBUG = 1 # 0: no debug 1: debug
    with open('/dev/BeuatoCtrl0', mode='r+b', buffering=0) as dev:
        import fcntl
        #fcntl.ioctl(dev, 0x40044200, struct.pack("I",DRIVER_DEBUG)) # debug dmesg
        fcntl.ioctl(dev, IOCTL_DEBUG, struct.pack("I", DRIVER_DEBUG))
        fcntl.ioctl(dev, IOCTL_READ_MODE, struct.pack("I", 0))


        while True:
            print("----------------------------")
            mode = input("mode: r(ead) / w(rite) / s(tring) : ")
            if mode == 'r':
                name = input("name = ")
                print(name)
                addr, length, vartype = memory_map[name]
                print(f'name: {name}, addr: {addr}, length: {length}, type: {vartype}')
                command = make_read_command_from_addr(addr, length, "r")
            elif mode == 'ra':
                addr, length = map(int, input("addr, length = ").split())
                print(f'addr: {addr}, length: {length}')
                command = make_read_command_from_addr(addr, length, "r")
            elif mode == 'w':
                #print("write mode not yet implemented.")
                name, data = input("name, data: ").split()
                addr, length, vartype = memory_map[name]
                print(f'name: {name}, addr: {addr}, length: {length}, data: {data}, type: {vartype}')
                command = make_write_command(name, data, "w")
                print(f"command: {command}")
            elif mode == 's':
                s = input("command string: ")
                command = make_command_from_str(s)
            else :
                print( 'mode is either "r" or "w"' )
                continue
            try:
                #addr = 0x0
                #length = 2

                res = send_and_receive_command_only(command)
                payload, size = parse_beuato_ascii(res)
                print(f"payload:{payload} size:{size}")

                #body_angle,timing1 = send_and_receive("BODY_ANGLE", "r")
                #print(f"BODY_ANGLE: {body_angle}")

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
