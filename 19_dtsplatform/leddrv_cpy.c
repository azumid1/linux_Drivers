#include<linux/types.h>
#include<linux/kernel.h>
#include<linux/delay.h>
#include<linux/ide.h>
#include<linux/init.h>
#include<linux/module.h>
#include<linux/errno.h>
#include<linux/gpio.h>
#include<asm/mach/map.h>
#include<asm/uaccess.h>
#include<asm/io.h>
#include<linux/cdev.h>
#include<linux/fs.h>
#include<linux/device.h>
#include<linux/of.h>
#include<linux/of_address.h>
#include<linux/of_irq.h>
#include<linux/of_gpio.h>
#include<linux/fcntl.h>
#include<linux/platform_device.h>

#define GPIOLEDNAME	"dtsplatled"
#define GPIOLEDCNT	1
#define LEDOFF		0
#define LEDON		1

struct gpioled_dev{
	dev_t devid;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	struct device_node *nd;
	int major;
	int minor;
	int led_gpio;
};

struct gpioled_dev gpioled;

static int gpioled_open(struct inode *inode, struct file *filp){
	filp->private_data = &gpioled;
	return 0;
}

static ssize_t gpioled_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt){
	struct gpioled_dev *dev = (struct gpioled_dev*)filp->private_data;
	unsigned char databuf[1];
	unsigned char ledsta;
	int ret;
	ret = copy_from_user(databuf, buf, cnt);
	ledsta = databuf[0];
	if(ledsta == LEDON){
		gpio_set_value(dev->led_gpio, 0);
	} else if(ledsta == LENOFF){
		gpio_set_value(dev->led_gpio, 1);
	}
	return 0;

}

static int gpioled_release(struct inode *inode, struct file *filp){
	return 0;
}

static struct file_operations gpioled_fops = {
	.owner = THIS_MODULE,
	.open = gpioled_open,
	.write = gpioled_write,
	.release = gpioled_release,
};

struct of_device_id led_of_match[] = {
	{.compatible="atkalpha-gpioled"},
	{},
};

static int led_probe(struct platform_device *dev){
	int ret;
	printk("led_probe!\r\n");

	/* 1.获取设备节点 */
	/* 驱动和设备匹配成功后，设备信息就会从设备树节点转换成platform_device结构体 */
	gpioled.nd = dev->dev.of_node;
	gpioled.led_gpio = of_get_named_gpio(gpioled.nd, "led-gpio", 0);
	printk("led-gpio num=%d\r\n", gpioled.led_gpio);

	ret = gpio_request(gpioled.led_gpio, "led-gpio");
	ret = gpio_direction_output(gpioled.led_gpio, 1);

	if(gpioled.major){
		gpioled.devid = MKDEV(gpioled.major, 0);
		ret = register_chrdev_region(gpioled.devid, GPIOLEDCNT, GPIOLEDNAME);
	} else{
		ret = alloc_chrdev_region(&gpioled.devid, 0, GPIOLEDCNT, GPIOLEDNAME);
		gpioled.major = MAJOR(gpioled.devid);
		gpioled.minor = MINOR(gpioled.devid);
	}

	gpioled.cdev.owner = THIS_MODULE;
	cdev_init(&gpioled.cdev, &gpioled_fops);
	cdev_add(&gpioled.cdev, gpioled.devid, GPIOLEDCNT);

	gpioled.class = class_create(THIS_MODULE, GOUILEDNAME);
	gpioled.device = device_create(gpioled.class, NULL, gpioled.devid, NULL, GPIOLEDNAME);

	return 0;

}

static int led_remove(struct platform_deive *dev){
	printk("led_remove!\r\n");
	cdev_del(&gpioled.cdev);
	unregister_chrdev_region(gpioled.devid, GPIOLEDCNT);
	device_destroy(gpioled.class, gpioled.devid);
	class_destroy(gpioled.class);
	gpio_free(gpioled.led_gpio);
	return 0;
}

struct platform_driver led_dirver = {
	.driver = {
		.name = "imx6ull-led",
		.of_match_table = led_of_match,
	},
	.probe = led_probe,
	.remove = led_remove,
};

static int __init leddrv_init(void){
	platform_driver_register(&led_driver);
}

static void __eixt leddrv_exit(void){
	platform_driver_unregister(&led_driver);
}

module_init(leddrv_init);
module_exit(leddrv_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");