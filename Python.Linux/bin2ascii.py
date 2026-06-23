import time
import functools
#import latency_decorator as ld
from LatencyMeasurer import LatencyMeasurer as ld
from BeuatoMemMap import memory_map
import struct

IOCTL_DEBUG = 0x40044200
IOCTL_READ_MODE = 0x40044201

if __name__ == '__main__':
    DRIVER_DEBUG = 1 # 0: no debug 1: debug
    with open('/dev/BeuatoCtrl0', mode='r+b', buffering=0) as dev:
        import fcntl
        fcntl.ioctl(dev, 0x40044200, struct.pack("I",DRIVER_DEBUG)) # debug dmesg

        while True:
            try:
                print("----------------------------")
                b = dev.read(64)
                for c in b:
                    print(f'{c:x} ',end="")
                print("")
            except ValueError as e:
                print(e)
                continue

            except KeyboardInterrupt:
                print("\n計測を中断しました")
                ld.print_stats()
                break
