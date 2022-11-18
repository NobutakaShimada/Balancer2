#pragma once

#define DMESG_INFO(fmt, ...) printk(KERN_INFO "[+] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__)
#define DMESG_ERR(fmt, ...) printk(KERN_ERR "[!] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__)

