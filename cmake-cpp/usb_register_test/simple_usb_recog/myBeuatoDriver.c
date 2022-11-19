#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h> // kmalloc

#include "config.h"
#include "logging.h"
//#include "typedefs.h"

#define VENDOR_ID 0x1962
#define PRODUCT_ID 0x2080

#define MINOR_BASE 192			// マイナー番号

int skel_probe(struct usb_interface* ip, const struct usb_device_id* pID);
void skel_disconnect(struct usb_interface* ip);
int skel_open(struct inode *inode, struct file *file);
int skel_release(struct inode *inode, struct file *file);
ssize_t skel_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos);

struct usb_skel {
	struct usb_device* pDev;
	struct usb_interface* ip;
	u8 bulkInEndpointAddr;			// エンドポイントのアドレス(In)
	struct kref kref;
};

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
	usb_put_dev(pDev->pDev);
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
	DMESG_INFO("mydevice_write\n");
	return count;
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
	kref_init(&pDev->kref);							// 参照カウンタの初期化
	pDev->pDev = usb_get_dev(interface_to_usbdev(ip));		// USBデバイスの存在チェック
	pDev->ip = ip;

	struct usb_host_interface* pHostIf = ip->cur_altsetting;

	// 0番目のエンドポイントの取得
	struct usb_endpoint_descriptor* ep = &pHostIf->endpoint[0].desc;
	pDev->bulkInEndpointAddr = ep->bEndpointAddress;

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
