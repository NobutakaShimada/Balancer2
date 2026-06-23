import hid

VID = 0x1962
PID = 0x2080

dev = hid.device()
dev.open(VID, PID)

print("manufacturer:", dev.get_manufacturer_string())
print("product:", dev.get_product_string())
print("serial:", dev.get_serial_number_string())

dev.close()

