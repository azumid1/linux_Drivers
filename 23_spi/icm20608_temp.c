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
#include<linux/input.h>
#include<linux/spi/spi.h>
#include<linux/delay.h>

#define ICM20608_CNT     1
#define ICM20608_NAME    "icm20608"

struct icm20608_dev{
    int major;
    int minor;
    dev_t devid;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    void *private_data;

};

static struct icm20608_dev icm20608dev;

static int icm20608_open(struct inode *inode, struct file *filp){
    return 0;
}

static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
    return 0;
}

static int icm20608_release(struct inode *inode, struct file *filp){
    return 0;
}

static struct file_operations icm20608_fops = {
    .owner = THIS_MODULE,
    .open = icm20608_open,
    .read = icm20608_read,
    .release = icm20608_release,
};


static int icm20608_probe(struct spi_device *spi){
    int ret = 0;
    printk("icm20608_probe\r\n");
    
    /* 搭建字符设备框架 */
    /* 1、创建设备号 */
	if (icm20608dev.major) {		/*  定义了设备号 */
		icm20608dev.devid = MKDEV(icm20608dev.major, 0);
		ret = register_chrdev_region(icm20608dev.devid, ICM20608_CNT, ICM20608_NAME);
	} else {						/* 没有定义设备号 */
		ret = alloc_chrdev_region(&icm20608dev.devid, 0, ICM20608_CNT, ICM20608_NAME);	/* 申请设备号 */
		icm20608dev.major = MAJOR(icm20608dev.devid);	/* 获取分配号的主设备号 */
		icm20608dev.minor = MINOR(icm20608dev.devid);	/* 获取分配号的次设备号 */
	}
    if(ret < 0){
        goto fail_devid;
    }
	printk("icm20608 major=%d,minor=%d\r\n",icm20608dev.major, icm20608dev.minor);	
	
	/* 2、初始化cdev */
	icm20608dev.cdev.owner = THIS_MODULE;
	cdev_init(&icm20608dev.cdev, &icm20608_fops);
	
	/* 3、添加一个cdev */
	ret = cdev_add(&icm20608dev.cdev, icm20608dev.devid, ICM20608_CNT);
    if(ret < 0){
        goto fail_cdev;
    } 
	/* 4、创建类 */
	icm20608dev.class = class_create(THIS_MODULE, ICM20608_NAME);
	if(IS_ERR(icm20608dev.class)){
        ret = PTR_ERR(icm20608dev.class);
        goto fail_classcrt;
    }

	/* 5、创建设备 */
	icm20608dev.device = device_create(icm20608dev.class, NULL, icm20608dev.devid, NULL, ICM20608_NAME);
	if(IS_ERR(icm20608dev.device)){
        ret = PTR_ERR(icm20608dev.device);
        goto fail_devicecrt;
    }

    /* 设置私有数据 */
    icm20608dev.private_data = spi;
    return 0;
fail_devicecrt:
    class_destroy(icm20608dev.class);
fail_classcrt:
    cdev_del(&icm20608dev.cdev);
fail_cdev:
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);
fail_devid:
    return ret;

}

static int icm20608_remove(struct spi_device *spi){
    cdev_del(&icm20608dev.cdev);
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);

    device_destroy(icm20608dev.class, icm20608dev.devid);
    class_destroy(icm20608dev.class);
    return 0;
}


/* 设备树匹配表 */
static const struct of_device_id icm20608_of_match[] = {
    {.compatible = "alientek,icm20608"},
    {}
};

/* 传统的匹配表 */
static struct spi_device_id icm20608_id[] = {
    {"alientek,icm20608",0},
    {}
};


/* spi_driver */
struct spi_driver icm20608drv = {
    .probe = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "icm20608",
        .of_match_table = icm20608_of_match,
    },
    .id_table = icm20608_id,

};

static int __init icm20608_init(void){
    int ret = 0;
    ret = spi_register_driver(&icm20608drv);
    return ret;
}

static void __exit icm20608_exit(void){
   spi_unregister_driver(&icm20608drv);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
