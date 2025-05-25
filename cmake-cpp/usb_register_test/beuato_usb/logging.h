#pragma once

#define DMESG_INFO(fmt, ...) \
	{ \
		printk(KERN_INFO "[+] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__); \
	}
#define DMESG_DEBUG(fmt, ...) \
	if(atomic_read(&beuato_dbg)) { \
		printk(KERN_DEBUG "[+] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__); \
	}
#define DMESG_DEBUG_DUMP(fmt, ptr, len, ascii, ...) \
        if(atomic_read(&beuato_dbg)) { \
            printk(KERN_DEBUG "[+] %s: %s: \n" fmt, DRIVER_DESC, __func__, ## __VA_ARGS__); \
            print_hex_dump(KERN_DEBUG, "dump: ", DUMP_PREFIX_OFFSET, 16, 1, ptr, len, ascii); \
        }
#define DMESG_WARN(fmt, ...) printk(KERN_WARNING "[!] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__)
#define DMESG_ERR(fmt, ...) printk(KERN_ERR "[!] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__)

