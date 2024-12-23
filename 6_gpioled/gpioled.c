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

#define GPIOLEDNAME      "gpioled"
#define GPIOLEDCNT        1
#define LEDOFF           0
#define LEDON            1

/* dtsled结构体 */
struct gpioled_dev{
    struct cdev cdev;           /* 字符设备     */
    struct class *class;        /* 类           */
    struct device *device;      /* 设备         */
    dev_t devid;                /* 设备号       */
    int major;                  /* 主设备号     */
    int minor;                  /* 次设备号     */
    struct device_node *nd;     /* 节点信息     */
    int led_gpio;               /*  led所使用的GPIO编号 */
};

struct gpioled_dev gpioled;

static int gpioled_open(struct inode *inode, struct file *filp){
    filp->private_data = &gpioled;       /*  这是设置了文件私有数据  */
    return 0;
}

static ssize_t gpioled_write(struct file *filp, const char __user *buf,
                            size_t cnt, loff_t *offt)
{
    //struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;
    int ret;
    unsigned char databuf[1];
    unsigned char ledstat;
    struct gpioled_dev *dev = (struct gpioled*)filp->private_data;

    ret = copy_from_user(databuf, buf, cnt);
    if(ret < 0){
        printk("kernle write failed\r\n");
        return -EFAULT;
    }
    ledstat = databuf[0];
    if(ledstat == LEDON){
        gpio_set_value(dev->led_gpio, 0);
    } else if(ledstat == LEDOFF){
        gpio_set_value(dev->led_gpio, 1);
    }
    return 0;

}
static int gpioled_release(struct inode *inode, struct file *filp){
    //struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;    /*  这是提取了私有数据  */
    return 0;
}

static struct file_operations gpioled_fops = {
    .owner = THIS_MODULE,
    .open = gpioled_open,
    .write = gpioled_write,
    .release = gpioled_release,
};

static int __init gpioled_init(void)
{
    int ret;
    gpioled.nd = NULL;

    /*  设置LED所使用的GPIO */
    /*  1.获取设备节点  */
    gpioled.nd = of_find_node_by_path("/gpioled");
    if(gpioled.nd == NULL){
        printk("gpioled node can not found!\r\n");
        return -EINVAL;
    } else {
        printk("gpioled node has been found!\r\n");
    }

    /*  2.获取设备树中的gpio属性，得到LED所使用的LED编号    */
    gpioled.led_gpio = of_get_named_gpio(gpioled.nd, "led-gpio", 0);
    //gpioled.led_gpio = of_get_named_gpio(gpioled.nd, "led-gpio", 0);
    if(gpioled.led_gpio < 0){
        printk("can't get led-gpio");
        return -EINVAL;
    }
    printk("led-gpio num = %d\r\n",gpioled.led_gpio);

    /* 3.申请一下GPIO管脚 名字随便起*/
    ret = gpio_request(gpioled.led_gpio, "led-gpio");
    if(ret){
        printk("failed to request the led gpio\r\n");
        ret = -EINVAL;
        goto fail_findnd;
    }

    /*  4.设置GPIO1_IO03为输出，并且输出高电平，默认关闭LED灯   */
    ret = gpio_direction_output(gpioled.led_gpio, 1);
    if(ret < 0){
        printk("can't set gpio!\r\n");
        goto fail_setoutput;
    }

    /* 注册新字符设备号 */
    gpioled.major = 0;       /* 设备号由内核分配 */
    if(gpioled.major){       /* 设备号已经给定   */
        gpioled.devid = MKDEV(gpioled.major, 0);
        ret = register_chrdev_region(gpioled.devid, GPIOLEDCNT, GPIOLEDNAME);
    } else {                /* 没有给定设备号   */
        ret = alloc_chrdev_region(&gpioled.devid, 0, GPIOLEDCNT, GPIOLEDNAME);
        gpioled.major = MAJOR(gpioled.devid);
        gpioled.minor = MINOR(gpioled.devid);
    }

    if(ret < 0){
        goto fail_devid;
    }

    /* 添加字符设备 */
    gpioled.cdev.owner = THIS_MODULE;
    cdev_init(&gpioled.cdev, &gpioled_fops);
    ret = cdev_add(&gpioled.cdev, gpioled.devid, GPIOLEDCNT);
    if(ret < 0){
        goto fail_cdevadd;
    } 

    /* 自动创建设备节点 */
    gpioled.class = class_create(THIS_MODULE, GPIOLEDNAME);
    if(IS_ERR(gpioled.class)){
        ret = PTR_ERR(gpioled.class);
        goto fail_classcrt;
    }

    gpioled.device = device_create(gpioled.class, NULL, gpioled.devid, NULL, GPIOLEDNAME);
    if(IS_ERR(gpioled.device)){
        ret = PTR_ERR(gpioled.device);
        goto fail_devicecrt;
    }

    return 0;



    device_destroy(gpioled.class, gpioled.devid);
fail_devicecrt:
    class_destroy(gpioled.class);
fail_classcrt:
    cdev_del(&gpioled.cdev);
fail_cdevadd:
    unregister_chrdev_region(gpioled.devid, GPIOLEDCNT);
fail_setoutput:
    gpio_free(gpioled.led_gpio);
fail_findnd:
fail_devid:
    return ret;
}

static void __exit gpioled_exit(void)
{
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
}

/* 注册和卸载驱动 */
module_init(gpioled_init);
module_exit(gpioled_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");