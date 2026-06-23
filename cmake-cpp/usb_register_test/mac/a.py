import usb.core
import usb.util

VID = 0x1962
PID = 0x2080

dev = usb.core.find(idVendor=VID, idProduct=PID)
print(dev)

if dev is None:
    raise RuntimeError("Balancer2 not found")
    
