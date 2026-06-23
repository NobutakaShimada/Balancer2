import hid

VID = 0x1962
PID = 0x2080

for d in hid.enumerate(VID, PID):
    print(d)
    
