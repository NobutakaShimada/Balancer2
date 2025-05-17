#!/bin/bash
dev="$1"
echo -n "$dev" > /sys/bus/usb/drivers/usbhid/unbind
echo -n "$dev" > /sys/bus/usb/drivers/BeuatoBalancer_Driver/bind

