#!/usr/bin/env python3
import struct
import sys
import math

def float_to_hex(value):
    """floatを16進表示（32bit IEEE 754）"""
    # ビッグエンディアン
    big_endian = struct.pack('>f', value)
    # リトルエンディアン  
    little_endian = struct.pack('<f', value)
    
    print(f"float (32bit):")
    print(f"  Big endian:    {big_endian.hex().upper()}")
    print(f"  Little endian: {little_endian.hex().upper()}")
    print(f"  Bytes:         {' '.join(f'{b:02X}' for b in big_endian)}")
    
    return big_endian

def double_to_hex(value):
    """doubleを16進表示（64bit IEEE 754）"""
    # ビッグエンディアン
    big_endian = struct.pack('>d', value)
    # リトルエンディアン
    little_endian = struct.pack('<d', value)
    
    print(f"double (64bit):")
    print(f"  Big endian:    {big_endian.hex().upper()}")
    print(f"  Little endian: {little_endian.hex().upper()}")
    print(f"  Bytes:         {' '.join(f'{b:02X}' for b in big_endian)}")
    
    return big_endian

def float_to_binary(value):
    """floatのバイナリ表示（IEEE 754解析付き）"""
    data = struct.pack('>f', value)
    bits = int.from_bytes(data, 'big')
    
    # ビット分解
    sign = (bits >> 31) & 0x1
    exponent = (bits >> 23) & 0xFF
    mantissa = bits & 0x7FFFFF
    
    # バイナリ文字列作成
    binary_str = f"{bits:032b}"
    formatted = f"{binary_str[0]} {binary_str[1:9]} {binary_str[9:]}"
    
    print(f"float binary:")
    print(f"  {formatted}")
    print(f"  S EEEEEEEE MMMMMMMMMMMMMMMMMMMMMMM")
    print(f"  Sign: {sign}")
    print(f"  Exponent: {exponent} (bias 127) = {exponent-127}")
    print(f"  Mantissa: {mantissa} = 1.{mantissa/2**23:.6f}")

def double_to_binary(value):
    """doubleのバイナリ表示（IEEE 754解析付き）"""
    data = struct.pack('>d', value)
    bits = int.from_bytes(data, 'big')
    
    # ビット分解
    sign = (bits >> 63) & 0x1
    exponent = (bits >> 52) & 0x7FF
    mantissa = bits & 0xFFFFFFFFFFFFF
    
    # バイナリ文字列作成
    binary_str = f"{bits:064b}"
    formatted = f"{binary_str[0]} {binary_str[1:12]} {binary_str[12:]}"
    
    print(f"double binary:")
    print(f"  {formatted}")
    print(f"  S EEEEEEEEEEE MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM")
    print(f"  Sign: {sign}")
    print(f"  Exponent: {exponent} (bias 1023) = {exponent-1023}")
    print(f"  Mantissa: {mantissa}")

def analyze_special_values(value):
    """特殊値の解析"""
    if math.isnan(value):
        print("特殊値: NaN (Not a Number)")
    elif math.isinf(value):
        if value > 0:
            print("特殊値: +Infinity")
        else:
            print("特殊値: -Infinity")
    elif value == 0.0:
        if math.copysign(1.0, value) == 1.0:
            print("特殊値: +0.0")
        else:
            print("特殊値: -0.0")

def convert_number(number_str):
    """文字列を数値に変換して解析"""
    try:
        # 特殊値の処理
        if number_str.lower() in ['inf', 'infinity', '+inf', '+infinity']:
            value = float('inf')
        elif number_str.lower() in ['-inf', '-infinity']:
            value = float('-inf')
        elif number_str.lower() in ['nan']:
            value = float('nan')
        else:
            value = float(number_str)
        
        print(f"入力: {number_str}")
        print(f"値: {value}")
        print("=" * 60)
        
        # 特殊値の解析
        analyze_special_values(value)
        print()
        
        # 16進表示
        float_to_hex(value)
        print()
        double_to_hex(value)
        print()
        
        # バイナリ解析
        float_to_binary(value)
        print()
        double_to_binary(value)
        
    except ValueError:
        print(f"エラー: '{number_str}' は有効な数値ではありません")

def interactive_mode():
    """対話モード"""
    print("IEEE 754 浮動小数点変換ツール")
    print("使用例: 3.14159, -1.23e-4, inf, -inf, nan")
    print("終了: Ctrl+C または 'quit'")
    print("-" * 50)
    
    while True:
        try:
            user_input = input("数値を入力: ").strip()
            if user_input.lower() in ['quit', 'exit', 'q']:
                break
            if user_input:
                convert_number(user_input)
                print("\n" + "-" * 50)
        except KeyboardInterrupt:
            print("\n終了します")
            break
        except EOFError:
            break

def main():
    if len(sys.argv) == 1:
        # 引数がない場合は対話モード
        interactive_mode()
    elif len(sys.argv) == 2:
        # 引数がある場合はそれを変換
        convert_number(sys.argv[1])
    else:
        print("使用法:")
        print(f"  {sys.argv[0]}           # 対話モード")
        print(f"  {sys.argv[0]} <数値>    # 単発変換")
        print()
        print("例:")
        print(f"  {sys.argv[0]} 3.14159")
        print(f"  {sys.argv[0]} -1.23e-4")
        print(f"  {sys.argv[0]} inf")

if __name__ == "__main__":
    main()

