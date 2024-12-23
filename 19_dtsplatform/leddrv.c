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

#define GPIOLEDNAME		"dtsplatled"
#define GPIOLEDCNT		1
#define LEDOFF			0
#define LEDON			1

struct gpioled_dev{
	dev_t devid;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	int major;
	int minor;
	struct device_node *nd;
	int led_gpio;
};

struct gpioled_dev gpioled;


#if 0
struct platform_driver {
	int (*probe)(struct platform_device *);
	int (*remove)(struct platform_device *);
	void (*shutdown)(struct platform_device *);
	int (*suspend)(struct platform_device *, pm_message_t state);
	int (*resume)(struct platform_device *);
	struct device_driver driver;
	const struct platform_device_id *id_table;
	bool prevent_deferred_probe;
};

#endif

#if 0
struct device_driver {
	const char		*name;
	struct bus_type		*bus;

	struct module		*owner;
	const char		*mod_name;	/* used for built-in modules */

	bool suppress_bind_attrs;	/* disables bind/unbind via sysfs */

	const struct of_device_id	*of_match_table;
	const struct acpi_device_id	*acpi_match_table;

	int (*probe) (struct device *dev);
	int (*remove) (struct device *dev);
	void (*shutdown) (struct device *dev);
	int (*suspend) (struct device *dev, pm_message_t state);
	int (*resume) (struct device *dev);
	const struct attribute_group **groups;

	const struct dev_pm_ops *pm;

	struct driver_private *p;
};

#endif

#if 0

struct of_device_id {
	char	name[32];
	char	type[32];
	char	compatible[128];
	const void *data;
};

#endif

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
	if(ret < 0){
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}
	ledsta = databuf[0];
	if(ledsta == LEDON){
		gpio_set_value(dev->led_gpio, 0);
	} else if(ledsta == LEDOFF){
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

static int led_probe(struct platform_device *dev){
	int ret;
	printk("led_probe!\r\n");
	
	/*  设置LED所使用的GPIO */
    /*  1.获取设备节点  */
#if 0
	gpioled.nd = of_find_node_by_path("/gpioled");
	if(gpioled.nd == NULL){
		printk("gpioled node can't found!\r\n");
		return -EINVAL;
	} else printk("gpioled node has been found!\r\n");
#endif
	/* 驱动和设备匹配成功后，设备信息就会从设备树节点转换成platform_device结构体 */
	gpioled.nd = dev->dev.of_node;		/* 从platform_device结构体中可以直接获取设备信息 */
	/*  2.获取设备树中的gpio属性，得到LED所使用的LED编号 */
	gpioled.led_gpio = of_get_named_gpio(gpioled.nd, "led-gpio", 0);
	if(gpioled.led_gpio < 0){
		printk("can't get led-gpio!\r\n");
		return -EINVAL;
	}
	printk("led-gpio num = %d\r\n",gpioled.led_gpio);

	/* 3.申请一下GPIO管脚 名字随便起*/
	ret = gpio_request(gpioled.led_gpio, "led-gpio");
	if(ret){
		printk("failed to request the led-gpio!\r\n");
		ret = -EINVAL;
		goto fail_findnd;
	}

	/*  4.设置GPIO1_IO03为输出，并且输出高电平，默认关闭LED灯   */
	ret = gpio_direction_output(gpioled.led_gpio, 1);
	if(ret < 0){
		printk("can't set gpio!\r\n");
		goto fail_set;
	}

	/* 注册新字符设备号 */
	if(gpioled.major){
		gpioled.devid = MKDEV(gpioled.major, 0);
		ret = register_chrdev_region(gpioled.devid, GPIOLEDCNT, GPIOLEDNAME);
	} else{
		ret = alloc_chrdev_region(&gpioled.devid, 0, GPIOLEDCNT, GPIOLEDNAME);
		gpioled.major = MAJOR(gpioled.devid);
		gpioled.minor = MINOR(gpioled.devid);
	}
	if(ret < 0) goto fail_devid;

	/* 添加字符设备 */
	gpioled.cdev.owner = THIS_MODULE;
	cdev_init(&gpioled.cdev, &gpioled_fops);
	ret = cdev_add(&gpioled.cdev, gpioled.devid, GPIOLEDCNT);
	if(ret < 0) goto fail_cdev;

	/* 自动创建设备节点 */
	gpioled.class = class_create(THIS_MODULE, GPIOLEDNAME);
	if(IS_ERR(gpioled.class)){
		ret = PTR_ERR(gpioled.class);
		goto fail_class;
	}

	gpioled.device = device_create(gpioled.class, NULL, gpioled.devid, NULL, GPIOLEDNAME);
	if(IS_ERR(gpioled.device)){
		ret = PTR_ERR(gpioled.device);
		goto fail_device;
	}

	return 0;

fail_device:
	class_destroy(gpioled.class);
fail_class:
	cdev_del(&gpioled.cdev);
fail_cdev:
	unregister_chrdev_region(gpioled.devid, GPIOLEDCNT);
fail_devid:
fail_set:
	gpio_free(gpioled.led_gpio);
fail_findnd:
	return ret;

}

static int led_remove(struct platform_device *dev){
	printk("led_remove!\r\n");
	/* 删除字符设备 */
	cdev_del(&gpioled.cdev);

	/* 删除设备号   */
	unregister_chrdev_region(gpioled.devid, GPIOLEDCNT);

	/* 注销设备节点 */
	device_destroy(gpioled.class, gpioled.devid);

	/* 删除类       */
	class_destroy(gpioled.class);

	/* 释放IO */
	gpio_free(gpioled.led_gpio);

	return 0;
}

struct of_device_id led_of_match[] = {
	{.compatible = "atkalpha-gpioled"},
	{/* Sentinel */},
};

struct platform_driver led_driver = {
	.driver = {
		.name = "imx6ull-led",				/* 无设备树和设备进行匹配，驱动的名字 */
		.of_match_table = led_of_match,		/* 设备树匹配表 */
	},
	.probe = led_probe,
	.remove = led_remove,
};

static int __init ledrv_init(void){
    return platform_driver_register(&led_driver);
}

static void __exit leddrv_exit(void){
    platform_driver_unregister(&led_driver);
}   

module_init(ledrv_init);
module_exit(leddrv_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");