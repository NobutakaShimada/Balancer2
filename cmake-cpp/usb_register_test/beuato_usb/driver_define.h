#pragma once

#include <linux/usb.h>

struct usb_skel {
	struct usb_device* udev;
	struct usb_interface* ip;
    struct usb_anchor submitted;
	u8 bulk_in_endpointAddr;			// エンドポイントのアドレス(In)
    u8 bulk_out_endpointAddr;           // エンドポイントのアドレス(Out)
	struct kref kref;
    struct completion bulk_in_completion;
} ;
