#pragma once

#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

struct usb_skel {
	struct usb_device* udev;
	struct usb_interface* ip;
    struct usb_anchor submitted;
	struct kref kref;

    struct usb_endpoint_descriptor* int_in_endpoint;
    unsigned char* int_in_buffer;
    int int_in_buffer_length;
    struct urb* int_in_urb;
    dma_addr_t* int_in_dma;

    struct usb_endpoint_descriptor* int_out_endpoint;
    int int_out_buffer_length;
} ;


enum command_state 
{
    STATE_READ,
    STATE_INVALID
};

struct user_read_parameter 
{
    long address;
    long datasize;
} ;

struct user_read_result 
{
    unsigned long datasize;
    unsigned char* buffer;
    unsigned int buffer_length;
} ;

struct user_command 
{
    enum command_state command_state;
    struct user_read_parameter read_parameters;
} ;
