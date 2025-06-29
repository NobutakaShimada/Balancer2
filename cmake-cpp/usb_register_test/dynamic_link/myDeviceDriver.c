#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <asm/current.h>
#include <asm/uaccess.h>

static const unsigned int MINOR_BASE = 0;
static const unsigned int MINOR_NUM = 2;

static struct class* mydevice_class;

#define DRIVER_NAME "MyDevice"

static unsigned int mydevice_major;

static struct cdev mydevice_cdev;

static int mydevice_open(struct inode *inode, struct file *file)
{
	printk("mydevice_open");
	return 0;
}

static int mydevice_close(struct inode *inode, struct file *file)
{
	printk("mydevice_close");
	return 0;
}

static ssize_t mydevice_read(struct file *file, char __user *buff, size_t count, loff_t *f_pos)
{
	printk("mydevice_read");
	buff[0] = 'A';
	return 1;
}

static ssize_t mydevice_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos)
{
	printk("mydevice_write");
	return 1;
}

struct file_operations s_mydevice_fops = {
		.open = mydevice_open,
		.release = mydevice_close,
		.read = mydevice_read,
		.write = mydevice_write,
};

static int mydevice_init(void)
{

	int alloc_ret = 0;
	int cdev_err = 0;
	dev_t dev;

	printk("mydevice_init\n");

	// 空いているメジャー番号の確保
	alloc_ret = alloc_chrdev_region(&dev, MINOR_BASE, MINOR_NUM, DRIVER_NAME);
	if (alloc_ret != 0)
	{
		printk(KERN_ERR "alloc_chrdev_region = %d", alloc_ret);
		return -1;
	}

	// 取得したdev
	mydevice_major = MAJOR(dev);
	dev = MKDEV(mydevice_major, MINOR_BASE);

	// cdev構造体の初期化とシステムコールハンドラテーブルの登録
	cdev_init(&mydevice_cdev, &s_mydevice_fops);
	mydevice_cdev.owner = THIS_MODULE;

	// デバイスドライバをカーネルに登録
	cdev_err = cdev_add(&mydevice_cdev, dev, MINOR_NUM);
	if (cdev_err != 0)
	{
		printk(KERN_ERR "cdev_add = %d\n", cdev_err);
		unregister_chrdev_region(dev, MINOR_NUM);
		return -1;
	}
	
	// クラスとしてドライバを登録する
	mydevice_class = class_create(THIS_MODULE, "mydevice");
	if (IS_ERR(mydevice_class)) 
	{
		printk(KERN_ERR "class_create\n");
		cdev_del(&mydevice_cdev);
		unregister_chrdev_region(dev, MINOR_NUM);
		return -1;
	}

	// sys/class/mydevice/mydevice*を作成
	for(int minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) 
	{
		device_create(mydevice_class, NULL, MKDEV(mydevice_major, minor), NULL, "mydevice%d", minor);
	}

	return 0;
}

static void mydevice_exit(void)
{
	dev_t dev;
	printk("mydevice_exit\n");

	dev = MKDEV(mydevice_major, MINOR_BASE);

	for(int minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) 
	{
		device_destroy(mydevice_class, MKDEV(mydevice_major, minor));
	}

	cdev_del(&mydevice_cdev);

	unregister_chrdev_region(dev, MINOR_NUM);
}

module_init(mydevice_init);
module_exit(mydevice_exit);

MODULE_LICENSE("Dual BSD/GPL");
