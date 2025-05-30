#pragma once

#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

// for Debug log switch
#define BEUATO_IOC_SET_DEBUG _IOW('B', 0, int)

// for binary / ascii mode selection
#define BEUATO_IOC_SET_MODE _IOW('B', 1, int)

/* モードの定義 */
enum {
    BEUATO_MODE_ASCII  = 0,
    BEUATO_MODE_BINARY = 1,
};


struct usb_skel {
	struct usb_device* udev;
	struct usb_interface* ip;
	struct usb_anchor submitted;
	struct kref kref;
	bool disconnected;
	int mode; // mode 0: binary(default)  1: ascii

	struct usb_endpoint_descriptor* int_in_endpoint;
	unsigned char* int_in_buffer;
	int int_in_buffer_length;
	struct urb* int_in_urb;

	struct usb_endpoint_descriptor* int_out_endpoint;
	int int_out_buffer_length;

	size_t expected_len;
	size_t received_len;
	u8 bin_buf[1024]
} ;


enum command_state 
{
    STATE_READ,
    STATE_WRITE,
    STATE_INVALID
};

struct user_read_parameter 
{
    long address;
    long datasize;
    char senddata[256];
} ;

struct user_read_result 
{
    volatile unsigned char command;
    volatile unsigned int addr;
    volatile unsigned long datasize;
    volatile unsigned char* buffer;
    volatile unsigned int buffer_length;
    volatile unsigned int ready_to_return;
} ;

struct user_command 
{
    enum command_state command_state;
    struct user_read_parameter read_parameters;
} ;
