import time
import functools
#import latency_decorator as ld
from LatencyMeasurer import LatencyMeasurer as ld

# Define memory map dictionary: variable name → [address, byte length, type]
memory_map = {
    "Product_ID": [0, 2, "us"],
    "Version": [2, 1, "uc"],
    "MODE": [3, 1, "uc"],
    "M_CURRENT_L": [4, 2, "s"],
    "M_CURRENT_R": [6, 2, "s"],
    "T_CURRENT_L": [8, 2, "s"],
    "T_CURRENT_R": [10, 2, "s"],
    "CURRENT_OFFSET_L": [12, 2, "s"],
    "CURRENT_OFFSET_R": [14, 2, "s"],
    "BODY_ANGULAR_SPD": [16, 8, "d"],
    "BODY_ANGLE": [24, 8, "d"],
    "BODY_ANGULAR_SPD_OFFSET": [32, 8, "d"],
    "WHEEL_ANGULAR_SPD_L": [40, 8, "d"],
    "WHEEL_ANGULAR_SPD_R": [48, 8, "d"],
    "WHEEL_ANGLE_L": [56, 8, "d"],
    "WHEEL_ANGLE_R": [64, 8, "d"],
    "T_SPD_L": [72, 8, "d"],
    "T_SPD_R": [80, 8, "d"],
    "ENC_L": [88, 8, "ll"],
    "ENC_R": [96, 8, "ll"],
    "GYRO_DATA": [104, 2, "s"],
    "ADC_C_LA": [106, 2, "us"],
    "ADC_C_LB": [108, 2, "us"],
    "ADC_C_RA": [110, 2, "us"],
    "ADC_C_RB": [112, 2, "us"],
    "PAD_BTN": [114, 2, "us"],
    "PAD_AN_RX": [116, 1, "c"],
    "PAD_AN_RY": [117, 1, "c"],
    "PAD_AN_LX": [118, 1, "c"],
    "PAD_AN_LY": [119, 1, "c"],
    "USER_AREA1": [120, 8, "d"],
    "USER_AREA2": [128, 8, "d"],
    "USER_AREA3": [136, 8, "d"],
    "USER_AREA4": [144, 8, "d"],
    "USER_AREA5": [152, 8, "d"],
    "USER_AREA6": [160, 8, "d"],
    "USER_AREA7": [168, 8, "d"],
    "USER_AREA8": [176, 8, "d"],
    "CALIBRATION_CNT": [184, 2, "us"],
    "CALIBRATION_TIME": [186, 1, "uc"],
    "LOG_MODE": [187, 1, "uc"],
    "LOG_DIV": [188, 2, "us"],
    "LOG_CNT": [190, 2, "us"],
    "LOG_FLG1": [192, 8, "ull"],
    "LOG_FLG2": [200, 8, "ull"],
    "LOG_FLG3": [208, 8, "ull"],
    "GAIN_BODY": [216, 8, "d"],
    "GAIN_WHEEL": [224, 8, "d"],
    "GAIN_WHEEL_CORRELATION": [232, 8, "d"],
    "GAIN_BODY_ANGLE": [240, 8, "d"],
    "GAIN_BODY_ANGULAR_SPD": [248, 8, "d"],
    "GAIN_WHEELANGLE": [256, 8, "d"],
    "GAIN_WHEEL_ANGULAR_SPD": [264, 8, "d"],
    "GAIN_WHEEL_CORRELATION_ANGLE": [272, 8, "d"],
    "GAIN_WHEEL_CORRELATION_ANGULAR_SPD": [280, 8, "d"],
    "GAIN_CURRENT_P": [288, 8, "d"],
    "GAIN_CURRENT_I": [296, 8, "d"],
    "GAIN_GYRO_HPF_COF": [304, 8, "d"],
    "GAIN_OPTION1": [312, 8, "d"],
    "GAIN_OPTION2": [320, 8, "d"],
    "GAIN_OPTION3": [328, 8, "d"],
    "GAIN_OPTION4": [336, 8, "d"],
    "GAIN_OPTION5": [344, 8, "d"],
    "GAIN_OPTION6": [352, 8, "d"]
}

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
    DRIVER_DEBUG=1
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
                #time.sleep(0.001)
                body_angular_spd, timing2 = send_and_receive("BODY_ANGULAR_SPD", "r")
                print(f"BODY_ANGULAR_SPD: {body_angular_spd}")
                #time.sleep(0.001)

                wheel_angle_l, timing3 = send_and_receive("WHEEL_ANGLE_L", "r")
                print(f"WHEEL_ANGLE_L: {wheel_angle_l}")
                #time.sleep(0.001)

                gyro_data, timing4  = send_and_receive("GYRO_DATA", "r")
                print(f"GYRO_DATA: {gyro_data}")
                #time.sleep(0.001)

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
