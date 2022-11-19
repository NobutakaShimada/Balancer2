#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h> // kmalloc

#include "config.h"
#include "driver_define.h"
#include "logging.h"

#define VENDOR_ID 0x1962
#define PRODUCT_ID 0x2080

#define MINOR_BASE 192			// マイナー番号

int skel_probe(struct usb_interface* ip, const struct usb_device_id* pID);
void skel_disconnect(struct usb_interface* ip);
int skel_open(struct inode *inode, struct file *file);
int skel_release(struct inode *inode, struct file *file);
ssize_t skel_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos);

struct usb_device_id skel_table[] = {
	{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
	{}
};

struct usb_driver skel_driver = {
	.name = "BeuatoBalancer Driver",
	.id_table = skel_table,
	.probe = skel_probe,					// 最初に実行される関数
	.disconnect = skel_disconnect		// 切断時に実行される関数
};

// ファイルオペレーション
struct file_operations skel_fops = {
	.owner = THIS_MODULE,
	.write = skel_write,
	.open = skel_open,
	.release = skel_release,
};

struct usb_class_driver skel_class = {
	.name = "usb/BeuatoCtrl%d",
	.fops = &skel_fops,
	.minor_base = MINOR_BASE
};

// 破棄関数
void skel_dispose(struct kref* pKref) 
{
	struct usb_skel* pDev = container_of(pKref, struct usb_skel, kref);
	usb_put_dev(pDev->udev);
	kfree(pDev);			// 独自の構造体の破棄
}

int skel_open(struct inode *inode, struct file *file)
{
	struct usb_interface* pIntf = usb_find_interface(&skel_driver, iminor(inode));
	struct usb_skel* pDev = usb_get_intfdata(pIntf);
	kref_get(&pDev->kref);
	file->private_data = (void*)pDev;
	return 0;
}

int skel_release(struct inode *inode, struct file *file)
{
	struct usb_skel* pDev = file->private_data;
	kref_put(&pDev->kref, skel_dispose);
	return 0;
}

loff_t seek_space(const char* buff_from_user, size_t count, loff_t pos) 
{
	for(loff_t i = pos ; i < count; i ++ ) 
	{
		if(buff_from_user[i] == ' ') 
		{
			return i;
		}
	}

	return -1;
}


ssize_t skel_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos)
{
	/*
	const int MAX_RAW_TEXT_SIZE = 128;
	if(count >= MAX_RAW_TEXT_SIZE) 
	{
		DMESG_ERR("Data size is too large");
		return -1;
	}

	printk("extract start(%lld)", count);

	char user_raw_text[MAX_RAW_TEXT_SIZE];
	copy_from_user(user_raw_text, buff, count);

	loff_t ofs = seek_space(user_raw_text, count, 0);
	if( ofs < 0 )
	{
		DMESG_ERR("Extraction of space was failed");
		return -1;
	}

	char extract_text[MAX_RAW_TEXT_SIZE];
	memset(extract_text, 0, MAX_RAW_TEXT_SIZE);
	memcpy(extract_text, user_raw_text, ofs);	


	DMESG_INFO("mydevice_write(%lld):%s", ofs, extract_text);
	*/

	struct usb_skel* pDev = file->private_data;
	char data[64];
	memset(data, 0, 64);
	data[0] = 'r';
	data[3] = '4';


	char* transmit_buff;
	struct urb* urb_header;
	urb_header = usb_alloc_urb(0, GFP_KERNEL);
	if(!urb_header) 
	{
		DMESG_ERR("Memory allocation failed");
		return -1;
	}

	usb_init_urb(urb_header);
	DMESG_INFO("Initialized");

	transmit_buff = kmalloc(64, GFP_KERNEL);
	DMESG_INFO("Buffer Allocated");
	
	if(transmit_buff == NULL) 
	{
		goto skel_write_Error;
	}

	//copy_from_user(urb_header->transfer_buffer, transmit_buff, 64);	
	//DMESG_INFO("Copy data from user");

	/*
	usb_fill_bulk_urb(urb_header, pDev->pDev, 
		usb_sndbulkpipe(pDev->pDev, ))
	*/

	kfree(transmit_buff);
	DMESG_INFO("Free Buffer");

	usb_free_urb(urb_header);
	DMESG_INFO("Free Urb");

	DMESG_INFO("Finished");

	return count;

skel_write_Error:
	usb_free_coherent(pDev->udev, 64, transmit_buff, urb_header->transfer_dma);
	usb_free_urb(urb_header);
	return -1;


	/*
	usb_skel* pDev = fp->private_data;
	int written = usb_control_msg(pDev, 
		usb_sndctrlpipe(pDev->pDev, 0),
		0, (USB_DIR_OUT | USB_TYPE_VENDOR),
		data, 64, 100);
	*/
}

int skel_probe(struct usb_interface* ip, const struct usb_device_id* pID) 
{
	int errno = -ENOMEM;

	struct usb_skel* pDev = kmalloc(sizeof(struct usb_skel), GFP_KERNEL);
	if(!pDev)
	{
		DMESG_ERR("Out of memory.\n");
		goto L_Error;
	}
	memset(pDev, 0, sizeof(*pDev));

	init_usb_anchor(&pDev->submitted);
	init_completion(&pDev->bulk_in_completion);
	kref_init(&pDev->kref);							// 参照カウンタの初期化
	pDev->udev = usb_get_dev(interface_to_usbdev(ip));		// USBデバイスの存在チェック
	pDev->ip = ip;

	struct usb_host_interface* pHostIf = ip->cur_altsetting;

	// エンドポイントの取得
	// https://wiki.bit-hive.com/north/pg/usb%E3%83%89%E3%83%A9%E3%82%A4%E3%83%90
	for(int i = 0 ; i < pHostIf->desc.bNumEndpoints; ++i ) 
	{
		dev_info(&ip->dev, "Endpoint %d\n", i);

		struct usb_endpoint_descriptor* endpoint = &pHostIf->endpoint[i].desc;

		if(usb_endpoint_dir_in(endpoint)) 
		{
			pDev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev_info(&ip->dev, "[I] Endpoint count:%d\n", i);
		}

		if(usb_endpoint_dir_out(endpoint)) 
		{
			pDev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev_info(&ip->dev, "[O] Endpoint count:%d\n", i);
		}
	}

	usb_set_intfdata(ip, pDev);

	errno = usb_register_dev(ip, &skel_class);
	if(errno) 
	{
		DMESG_ERR("Not able to get minor for this device.\n");
		usb_set_intfdata(ip, NULL);
		goto L_Error;
	}

	dev_info(&ip->dev, "[+] usb_skel: Attached=%d", ip->minor);
	return 0;

L_Error:
	if(pDev) 
	{
		kref_put(&pDev->kref, skel_dispose);
	}
	return errno;
}

void skel_disconnect(struct usb_interface* ip) 
{
	dev_info(&ip->dev, "[+] usb_skel: (%d) is skel_disconnected", ip->minor);

	struct usb_skel* pDev = usb_get_intfdata(ip);
	usb_set_intfdata(ip, NULL);

	usb_deregister_dev(ip, &skel_class);
	kref_put(&pDev->kref, skel_dispose);
}

MODULE_DEVICE_TABLE(usb, skel_table);
module_usb_driver(skel_driver);

MODULE_LICENSE("Dual BSD/GPL");
