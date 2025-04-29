#!/bin/bash
rmmod usbhid
insmod /lib/modules/$(uname -r)/kernel/drivers/hid/usbhid/usbmouse.ko
insmod /lib/modules/$(uname -r)/kernel/drivers/hid/usbhid/usbkbd.ko

