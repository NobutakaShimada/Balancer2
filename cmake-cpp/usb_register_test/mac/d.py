import hid
import time

VID = 0x1962
PID = 0x2080

dev = hid.device()
dev.open(VID, PID)
dev.set_nonblocking(False)

# 既知のコマンド本体に置き換え
cmd = bytes([0x12, 0x34, 0x56])

# HID writeでは、多くの実装で先頭にReport IDが必要。
# Report IDなしなら 0x00 を付ける。
packet = bytes([0x00]) + cmd
packet = packet.ljust(65, b"\x00")  # Report ID 1 byte + payload 64 bytes

print("TX:", packet.hex())
n = dev.write(packet)
print("written:", n)

resp = dev.read(64, timeout_ms=1000)
print("RX:", bytes(resp).hex(), "len=", len(resp))

dev.close()

