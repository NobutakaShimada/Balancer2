import time

# Define memory map dictionary: variable name → [address, byte length]
memory_map = {
    "Product_ID": [0, 2],
    "Version": [2, 1],
    "MODE": [3, 1],
    "M_CURRENT_L": [4, 2],
    "M_CURRENT_R": [6, 2],
    "T_CURRENT_L": [8, 2],
    "T_CURRENT_R": [10, 2],
    "CURRENT_OFFSET_L": [12, 2],
    "CURRENT_OFFSET_R": [14, 2],
    "BODY_ANGULAR_SPD": [16, 8],
    "BODY_ANGLE": [24, 8],
    "BODY_ANGULAR_SPD_OFFSET": [32, 8],
    "WHEEL_ANGULAR_SPD_L": [40, 8],
    "WHEEL_ANGULAR_SPD_R": [48, 8],
    "WHEEL_ANGLE_L": [56, 8],
    "WHEEL_ANGLE_R": [64, 8],
    "T_SPD_L": [72, 8],
    "T_SPD_R": [80, 8],
    "ENC_L": [88, 8],
    "ENC_R": [96, 8],
    "GYRO_DATA": [104, 2],
    "ADC_C_LA": [106, 2],
    "ADC_C_LB": [108, 2],
    "ADC_C_RA": [110, 2],
    "ADC_C_RB": [112, 2],
    "PAD_BTN": [114, 2],
    "PAD_AN_RX": [116, 1],
    "PAD_AN_RY": [117, 1],
    "PAD_AN_LX": [118, 1],
    "PAD_AN_LY": [119, 1],
    "USER_AREA1": [120, 8],
    "USER_AREA2": [128, 8],
    "USER_AREA3": [136, 8],
    "USER_AREA4": [144, 8],
    "USER_AREA5": [152, 8],
    "USER_AREA6": [160, 8],
    "USER_AREA7": [168, 8],
    "USER_AREA8": [176, 8],
    "CALIBRATION_CNT": [184, 2],
    "CALIBRATION_TIME": [186, 1],
    "LOG_MODE": [187, 1],
    "LOG_DIV": [188, 2],
    "LOG_CNT": [190, 2],
    "LOG_FLG1": [192, 8],
    "LOG_FLG2": [200, 8],
    "LOG_FLG3": [208, 8],
    "GAIN_BODY": [216, 8],
    "GAIN_WHEEL": [224, 8],
    "GAIN_WHEEL_CORRELATION": [232, 8],
    "GAIN_BODY_ANGLE": [240, 8],
    "GAIN_BODY_ANGULAR_SPD": [248, 8],
    "GAIN_WHEELANGLE": [256, 8],
    "GAIN_WHEEL_ANGULAR_SPD": [264, 8],
    "GAIN_WHEEL_CORRELATION_ANGLE": [272, 8],
    "GAIN_WHEEL_CORRELATION_ANGULAR_SPD": [280, 8],
    "GAIN_CURRENT_P": [288, 8],
    "GAIN_CURRENT_I": [296, 8],
    "GAIN_GYRO_HPF_COF": [304, 8],
    "GAIN_OPTION1": [312, 8],
    "GAIN_OPTION2": [320, 8],
    "GAIN_OPTION3": [328, 8],
    "GAIN_OPTION4": [336, 8],
    "GAIN_OPTION5": [344, 8],
    "GAIN_OPTION6": [352, 8]
}

def make_command(var_name: str) -> bytes:
    """
    var_name に対応するアドレスとバイト長を
    "r addr, len " の形式で組み、ASCII バイト列で返す。
    例: var_name="BODY_ANGLE" → b"r 18, 8 "
    """
    try:
        addr, length = memory_map[var_name]
    except KeyError:
        raise ValueError(f"Unknown variable: {var_name!r}")
    # addr を 16 進文字列に（0xなし、小文字）
    addr_hex = format(addr, 'x')
    cmd = f"r {addr_hex} {length} "
    return cmd.encode('ascii'), length
    #return cmd, length



if __name__ == '__main__':
    with open('/dev/BeuatoCtrl0', mode='r+b', buffering=0) as dev:
        while True:
            #dev.write(b"r 68 2 ")
            #command = make_command("GYRO_DATA")
            command, len = make_command("BODY_ANGLE")
            print(f'{command}: ({len})')
            dev.write(command)
            dev.flush()
            #time.sleep(0.00001)
            res = dev.readline().strip()
            print(res)
