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

# 型コード → struct.unpack/pack 用フォーマット
TYPE_FMT = {
    "us": "<H",   # unsigned short
    "uc": "<B",   # unsigned char
    "c":  "<b",   # signed char
    "s":  "<h",   # short
    "d":  "<d",   # double
    "ll": "<q",   # long long
    "ull":"<Q",   # unsigned long long
}
