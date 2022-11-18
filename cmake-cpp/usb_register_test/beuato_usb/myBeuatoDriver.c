#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h> // kmalloc
#include "typedefs.h"

#define VENDOR_ID 0x1962
#define PRODUCT_ID 0x2080

#define MINOR_BASE 192			// マイナー番号

int Probe(UsbInterface* ip, const UsbDeviceID* pID);
void Disconnect(UsbInterface* ip);
int Open(struct inode *inode, struct file *file);
int Release(struct inode *inode, struct file *file);
ssize_t Write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos);

typedef struct {
	UsbDevice* pDev;
	UsbInterface* ip;
	u8 bulkInEndpointAddr;			// エンドポイントのアドレス
	Kref kref;
} BeuatoCtrl;

struct usb_device_id entries[] = {
	{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
	{}
};


struct usb_driver usb_driver = {
	.name = "BeuatoBalancer Driver",
	.id_table = entries,
	.probe = Probe,					// 最初に実行される関数
	.disconnect = Disconnect		// 切断時に実行される関数
};

// ファイルオペレーション
FileOperations usb_fops = {
	.owner = THIS_MODULE,
	.write = Write,
	.open = Open,
	.release = Release,
};

UsbClassDriver class = {
	.name = "usb/BeuatoCtrl%d",
	.fops = &usb_fops,
	.minor_base = MINOR_BASE
};

// 破棄関数
void Dispose(Kref* pKref) 
{
#define ToDev(d) container_of(d, BeuatoCtrl, kref)
	BeuatoCtrl* pDev = ToDev(pKref);
	usb_put_dev(pDev->pDev);
	kfree(pDev);			// 独自の構造体の破棄
}

int Open(struct inode *inode, struct file *file)
{
	UsbInterface* pIntf = usb_find_interface(&usb_driver, iminor(inode));
	BeuatoCtrl* pDev = usb_get_intfdata(pIntf);
	kref_get(&pDev->kref);
	file->private_data = (void*)pDev;
	return 0;
}

int Release(struct inode *inode, struct file *file)
{
	BeuatoCtrl* pDev = file->private_data;
	kref_put(&pDev->kref, Dispose);
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


ssize_t Write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos)
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

	BeuatoCtrl* pDev = file->private_data;
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

	transmit_buff = usb_alloc_coherent(pDev->pDev, 64, GFP_KERNEL, &urb_header->transfer_dma);
	DMESG_INFO("Buffer Allocated");



	usb_free_urb(urb_header);
	DMESG_INFO("Free Urb");

	usb_free_coherent(pDev->pDev, 64, transmit_buff, urb_header->transfer_dma);
	DMESG_INFO("Free Buffer");

	DMESG_INFO("Finished");

	return count;

Write_Error:
	usb_free_coherent(pDev->pDev, 64, transmit_buff, urb_header->transfer_dma);
	usb_free_urb(urb_header);
	return -1;


	/*
	BeuatoCtrl* pDev = fp->private_data;
	int written = usb_control_msg(pDev, 
		usb_sndctrlpipe(pDev->pDev, 0),
		0, (USB_DIR_OUT | USB_TYPE_VENDOR),
		data, 64, 100);
	*/
}

int Probe(UsbInterface* ip, const UsbDeviceID* pID) 
{
	int errno = -ENOMEM;

	BeuatoCtrl* pDev = kmalloc(sizeof(BeuatoCtrl), GFP_KERNEL);
	if(!pDev)
	{
		DMESG_ERR("Out of memory.\n");
		goto L_Error;
	}

	memset(pDev, 0, sizeof(*pDev));
	kref_init(&pDev->kref);							// 参照カウンタの初期化
	pDev->pDev = usb_get_dev(interface_to_usbdev(ip));		// USBデバイスの存在チェック
	pDev->ip = ip;

	UsbHostInterface* pHostIf = ip->cur_altsetting;

	// 0番目のエンドポイントの取得
	UsbEndpointDescriptor* ep = &pHostIf->endpoint[0].desc;
	pDev->bulkInEndpointAddr = ep->bEndpointAddress;

	usb_set_intfdata(ip, pDev);

	errno = usb_register_dev(ip, &class);
	if(errno) 
	{
		DMESG_ERR("Not able to get minor for this device.\n");
		usb_set_intfdata(ip, NULL);
		goto L_Error;
	}

	dev_info(&ip->dev, "[+] BeuatoCtrl: Attached=%d", ip->minor);
	return 0;

L_Error:
	if(pDev) 
	{
		kref_put(&pDev->kref, Dispose);
	}
	return errno;
}

void Disconnect(UsbInterface* ip) 
{
	dev_info(&ip->dev, "[+] BeuatoCtrl: (%d) is Disconnected", ip->minor);

	BeuatoCtrl* pDev = usb_get_intfdata(ip);
	usb_set_intfdata(ip, NULL);

	usb_deregister_dev(ip, &class);
	kref_put(&pDev->kref, Dispose);
}

MODULE_DEVICE_TABLE(usb, entries);
module_usb_driver(usb_driver);

MODULE_LICENSE("Dual BSD/GPL");
