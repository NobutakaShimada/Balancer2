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
ssize_t Read(struct file *file, char __user *buff, size_t count, loff_t *f_pos);
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

ssize_t Read(struct file *file, char __user *buff, size_t count, loff_t *f_pos)
{
	printk("mydevice_read");
	buff[0] = 'A';
	return 1;
}

ssize_t Write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos)
{
	printk("mydevice_write");
	return 1;
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
