#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/usb.h>
#include <linux/slab.h>

#include "config.h"
#include "driver_define.h"
#include "logging.h"

#define VENDOR_ID 0x1962
#define PRODUCT_ID 0x2080

#define MINOR_BASE 192			// マイナー番号

/** プロトタイプ宣言 **/ 
int skel_probe(struct usb_interface* ip, const struct usb_device_id* pID);
void skel_disconnect(struct usb_interface* ip);
int skel_open(struct inode *inode, struct file *file);
int skel_release(struct inode *inode, struct file *file);
ssize_t skel_read (struct file* file, char __user *buff, size_t count, loff_t *f_pos) ;
ssize_t skel_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos);

/** ベンダーIDとプロダクトIDの登録 **/ 
struct usb_device_id skel_table[] = {
	{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
	{}
};

/** ドライバ初期化時の処理の登録 **/
struct usb_driver skel_driver = {
	.name = "BeuatoBalancer Driver",
	.id_table = skel_table,
	.probe = skel_probe,					// 最初に実行される関数
	.disconnect = skel_disconnect		// 切断時に実行される関数
};


/** ファイルオペレーションの登録 **/
struct file_operations skel_fops = {
	.owner = THIS_MODULE,
	.open = skel_open,
	.release = skel_release,
	.write = skel_write,
};

/** ドライバクラスの登録 */
struct usb_class_driver skel_class = {
	.name = "usb/BeuatoCtrl%d",
	.fops = &skel_fops,
	.minor_base = MINOR_BASE
};


static void skel_dispose(struct kref* pKref) 
{
	struct usb_skel* pDev = container_of(pKref, struct usb_skel, kref);
	usb_put_dev(pDev->udev);
	kfree(pDev);
}

/* デバイスオープン時に実行 */
int skel_open(struct inode *inode, struct file *file)
{
	struct usb_interface* ip = usb_find_interface(&skel_driver, iminor(inode));
	struct usb_skel* pDev = usb_get_intfdata(ip);

	kref_get(&pDev->kref);
	file->private_data = (void*)pDev;

	return 0;
}


/* デバイスリリース時に実行 */
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


/** USB-HID-INの制御 
 * 
 * 
 * 
 * **/

/* データの解釈 */
void report_in_handler(u8 *buf, int length)
{
	DMESG_INFO("[I] Read Message:%d", length);
	printk("Data:");

	for(int i = 0 ; i < length; i ++) 
	{
		printk(KERN_CONT "%x ", buf[i]);			
	}

	DMESG_INFO("Report Handled");
}

/** HID-INデータ完了通知 **/
void urb_in_complete(struct urb* urb) 
{
	switch (urb->status) {
	case 0:
		report_in_handler(urb->transfer_buffer, urb->transfer_buffer_length);
		DMESG_INFO("Urb load rom Success");
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		DMESG_ERR("urb shutting down with %d\n", urb->status);
		break;
	default:
		DMESG_ERR("urb status %d received\n", urb->status);	
		break;
	}

	DMESG_INFO("Completed");
}

/** データ読み込み準備 **/
int prepare_read(struct usb_interface* ip, const struct usb_device_id* pID, struct usb_skel* pDev) 
{
	pDev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!pDev->int_in_urb) 
	{
		DMESG_ERR("Memory allocation failed");
		return -1;
	}

	pDev->int_in_buffer = kmalloc(pDev->int_in_buffer_length, GFP_KERNEL);
	memset(pDev->int_in_buffer, 0, pDev->int_in_buffer_length);
	DMESG_INFO("Allocation");

	return 0;
}

/** データ読み込み開始 **/
int run_read(struct usb_skel* pDev) 
{
	usb_fill_int_urb(pDev->int_in_urb, pDev->udev, 
		usb_rcvintpipe(pDev->udev, pDev->int_in_endpoint->bEndpointAddress),
		pDev->int_in_buffer,
		pDev->int_in_buffer_length,
		urb_in_complete,
		pDev,
		pDev->int_in_endpoint->bInterval
	);

	usb_submit_urb(pDev->int_in_urb, GFP_KERNEL);
	DMESG_INFO("Prepare read");

	return 0;
}

/** USB-HID-OUTの制御 
 * 
 * 
 * 
 * **/

/* データの解釈 */
void report_out_handler(u8 *buf, int length)
{
	switch (buf[0])
	{
		case 'r':
			DMESG_INFO("[O] Read Message");
			printk("Data:");
			for(int i = 0 ; i < length; i ++) 
			{
				printk(KERN_CONT "%x ", buf[i]);			
			}
			break;
		default:
			DMESG_INFO("Other Message:%c", buf[0]);
	}

	DMESG_INFO("Report Handled");
}

/** HID-OUTデータ完了通知 **/
void urb_out_complete(struct urb* urb) 
{
	switch (urb->status) {
	case 0:
		report_out_handler(urb->transfer_buffer, urb->transfer_buffer_length);
		//usb_submit_urb(urb, GFP_ATOMIC);
		DMESG_INFO("Urb Success");
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		DMESG_ERR("urb shutting down with %d\n", urb->status);
		break;
	default:
		DMESG_ERR("urb status %d received\n", urb->status);
		//usb_submit_urb(urb, GFP_ATOMIC);			
		break;
	}

	DMESG_INFO("Completed");
}

/** データ書き込み **/
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
	
	struct urb* urb_header;
	urb_header = usb_alloc_urb(0, GFP_KERNEL);
	if(!urb_header) 
	{
		DMESG_ERR("Memory allocation failed");
		return -1;
	}

	char* transmit_buff;
	transmit_buff = kmalloc(pDev->int_out_buffer_length, GFP_KERNEL);
	memset(transmit_buff, 0, pDev->int_out_buffer_length);
	transmit_buff[0] = 'r';
	transmit_buff[3] = 8;

	usb_fill_int_urb(urb_header, pDev->udev, 
		usb_sndintpipe(pDev->udev, pDev->int_out_endpoint->bEndpointAddress),
		transmit_buff,
		pDev->int_out_buffer_length,
		urb_out_complete,
		pDev,
		pDev->int_out_endpoint->bInterval
	);

	DMESG_INFO("Buffer Allocated");
	
	if(transmit_buff == NULL) 
	{
		goto skel_write_Error;
	}

	usb_submit_urb(urb_header, GFP_KERNEL);

	run_read(pDev);

	kfree(transmit_buff);
	DMESG_INFO("Free Buffer");

	usb_free_urb(urb_header);
	DMESG_INFO("Free Urb");
	
	return count;

skel_write_Error:
	usb_free_coherent(pDev->udev, pDev->int_out_buffer_length, transmit_buff, urb_header->transfer_dma);
	usb_free_urb(urb_header);
	return -1;
}


/**
 * USB接続開始
 * 
 * **/

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
	kref_init(&pDev->kref);							// 参照カウンタの初期化
	pDev->udev = usb_get_dev(interface_to_usbdev(ip));		// USBデバイスの存在チェック
	pDev->ip = ip;

	// エンドポイントの取得
	// https://wiki.bit-hive.com/north/pg/usb%E3%83%89%E3%83%A9%E3%82%A4%E3%83%90
	struct usb_host_interface* pHostIf = ip->cur_altsetting;
	for(int i = 0 ; i < pHostIf->desc.bNumEndpoints; ++i ) 
	{
		dev_info(&ip->dev, "Endpoint %d\n", i);

		struct usb_endpoint_descriptor* endpoint = &pHostIf->endpoint[i].desc;

		if(usb_endpoint_dir_in(endpoint)) 
		{
			pDev->int_in_endpoint = endpoint;
			pDev->int_in_buffer_length = endpoint->wMaxPacketSize;
			dev_info(&ip->dev, "[I] Endpoint count:%d\n", i);
		}

		if(usb_endpoint_dir_out(endpoint)) 
		{
			pDev->int_out_endpoint = endpoint;
			pDev->int_out_buffer_length = endpoint->wMaxPacketSize;
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

	errno = prepare_read(ip, pID, pDev);
	if(errno) 
	{
		DMESG_ERR("Failed to prepare reading buffer\n");
		goto L_Error;
	}

	return 0;
	//return run_read(pDev);
L_Error:
	if(pDev) 
	{
		kref_put(&pDev->kref, skel_dispose);
	}
	return errno;
}

/**
 * USB接続終了
 * 
 * **/

void skel_disconnect(struct usb_interface* ip) 
{
	dev_info(&ip->dev, "[+] usb_skel: (%d) is skel_disconnected", ip->minor);

	struct usb_skel* pDev = usb_get_intfdata(ip);
	usb_set_intfdata(ip, NULL);

	// dispose interrupt in buffer
	kfree(pDev->int_in_buffer);
	DMESG_INFO("Free Buffer");

	usb_free_urb(pDev->int_in_urb);
	DMESG_INFO("Free Urb");

	usb_deregister_dev(ip, &skel_class);
	kref_put(&pDev->kref, skel_dispose);
}

MODULE_DEVICE_TABLE(usb, skel_table);
module_usb_driver(skel_driver);

MODULE_LICENSE("Dual BSD/GPL");
