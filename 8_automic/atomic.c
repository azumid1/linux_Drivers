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
#define LEDOFF            0
#define LEDON             1

struct gpioled_dev{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    int major;
    int minor;
    struct device_node *nd;
    int led_gpio;
    atomic_t lock;         /*  原子变量    */

};

struct gpioled_dev gpioled;

static int gpioled_open(struct inode *inode, struct file *filp){
    /*  通过判断原子变量的值来检查LED有没有被别的应用使用   */
    if(!atomic_dec_and_test(&gpioled.lock)){
        atomic_inc(&gpioled.lock); /* 小于0的话就加1，使其原子变量等于0 */
        return -EBUSY;
    }
    filp->private_data = &gpioled;
    return 0;
}

static ssize_t gpioled_write(struct file *filp, const char __user *buf,
                            size_t cnt, loff_t *offt)
{
    int ret;
    unsigned char databuf[1];
    unsigned char ledstat;
    struct gpioled_dev *dev = (struct gpioled_dev*)filp->private_data;
    
    ret = copy_from_user(databuf, buf, cnt);
    if(ret < 0){
        printk("kernel write failed\r\n");
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
    struct gpioled_dev *dev = (struct gpioled_dev*)filp->private_data;

    /*  关闭驱动文件的时候释放原子变量 */
    atomic_inc(&dev->lock);
    return 0;
}

static struct file_operations gpioled_fops = {
    .owner = THIS_MODULE,
    .open =gpioled_open,
    .write = gpioled_write,
    .release = gpioled_release,
};

static int __init gpioled_init(void){
    int ret = 0;
    gpioled.nd = NULL;

    /*  初始化原子变量  */
    atomic_set(&gpioled.lock, 1);    /*  原子变量初始值为1   */

    /*  设置lED所使用的GPIO */
    /*  1.获取设备节点  */
    gpioled.nd = of_find_node_by_path("/gpioled");
    if(gpioled.nd == NULL){
        printk("gpioled node can not found!\r\n");
        return -EINVAL;
    } else{
        printk("gpioled node has been found!\r\n");
    }

    /*  2.获取设备树中的GPIO属性，得到LED所使用的LED编号    */
    gpioled.led_gpio = of_get_named_gpio(gpioled.nd, "led-gpio", 0);
    if(gpioled.led_gpio < 0){
        printk("can't get led-gpio\r\n");
        return -EINVAL;
    } else{
        printk("led-gpio num = %d",gpioled.led_gpio);
    }

    /*  3.申请GPIO管脚  */
    ret = gpio_request(gpioled.led_gpio, "led-gpio");
    if(ret){
        ret = -EINVAL;
        goto fail_fdnd;
    }

    /*  4.设置GPIO1_IO03为输出，并且输出高电平，默认关闭    */
    ret = gpio_direction_output(gpioled.led_gpio, 1);
    if(ret < 0){
        printk("can't set gpio!\r\n");
        goto fail_setoutput;
    }

    if(gpioled.major){
        gpioled.devid = MKDEV(gpioled.major, 0);
        ret = register_chrdev_region(gpioled.devid, GPIOLEDCNT, GPIOLEDNAME);
    } else{
        ret = alloc_chrdev_region(&gpioled.devid, 0, GPIOLEDCNT, GPIOLEDNAME);
        gpioled.major = MAJOR(gpioled.devid);
        gpioled.minor = MINOR(gpioled.devid);
    }

    if(ret < 0){
        goto fail_devid;
    }

    gpioled.cdev.owner = THIS_MODULE;
    cdev_init(&gpioled.cdev, &gpioled_fops);
    ret = cdev_add(&gpioled.cdev, gpioled.devid, GPIOLEDCNT);
    if(ret < 0){
        goto fail_cdevadd;
    }
 
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


fail_devicecrt:
    class_destroy(gpioled.class);
fail_classcrt:
    cdev_del(&gpioled.cdev);
fail_cdevadd:
    unregister_chrdev_region(gpioled.devid, GPIOLEDCNT);
fail_devid:
fail_setoutput:
    gpio_free(gpioled.led_gpio);
fail_fdnd:
    return ret;
}

static void __exit gpioled_exit(void){
    gpio_free(gpioled.led_gpio);
    cdev_del(&gpioled.cdev);
    unregister_chrdev_region(gpioled.devid, GPIOLEDCNT);
    device_destroy(gpioled.class, gpioled.devid);
    class_destroy(gpioled.class);
}

module_init(gpioled_init);
module_exit(gpioled_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
