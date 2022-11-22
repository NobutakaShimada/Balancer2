#pragma once

#include <linux/usb.h>

struct usb_skel {
	struct usb_device* udev;
	struct usb_interface* ip;
    struct usb_anchor submitted;
	struct kref kref;

    struct usb_endpoint_descriptor* int_in_endpoint;
    struct usb_endpoint_descriptor* int_out_endpoint;

} ;