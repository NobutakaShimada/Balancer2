
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>

static char* your_name = "Taro";
module_param(your_name, charp, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

static int return_value = 0;
module_param(return_value, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);


static int __init hello_world_init(void) 
{
  printk(KERN_INFO "%s: Hello world %s. return_value=%d\n", __func__, your_name, return_value);
  return return_value;
}

static void __exit hello_world_exit(void)
{
  printk(KERN_INFO "%s: Goodbye %s.\n", __func__, your_name) ;
}

module_init(hello_world_init);
module_exit(hello_world_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tutorial hello you");
MODULE_AUTHOR("Your name or email address");


