// Typedefs.h
#pragma once
 
typedef unsigned long ul;
typedef unsigned int uint;
typedef struct usb_interface UsbInterface;
typedef struct usb_device_id UsbDeviceID;
typedef struct usb_device UsbDevice;
typedef struct usb_anchor UsbAnchor;
typedef struct usb_host_interface UsbHostInterface;
typedef struct usb_endpoint_descriptor UsbEndpointDescriptor;
typedef struct usb_class_driver UsbClassDriver;
typedef struct usb_ctrlrequest  UsbCtrlRequest;
typedef struct semaphore Semaphore;
typedef struct urb Urb;
typedef struct kref Kref;
typedef struct mutex Mutex;
typedef struct completion Completion;
typedef struct inode Inode;
typedef struct file File;
typedef struct file_operations FileOperations;
typedef int (*PfIoctl)(Inode *pInode, File *pFile, uint cmd, ul arg);
 
#define DRIVER_DESC "LedCtrl" // ドライバの名前
// printf
#define DMESG_INFO(fmt, ...) printk(KERN_INFO "[+] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__)
#define DMESG_ERR(fmt, ...) printk(KERN_ERR "[!] %s: %s: " fmt, DRIVER_DESC, __func__, ## __VA_ARGS__)