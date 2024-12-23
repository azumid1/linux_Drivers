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

#define BEEPNAME         "beep"
#define BEEPCNT           1
#define BEEPOFF           0
#define BEEPON            1

struct beep_dev{
    struct cdev cdev;           /*  字符设备    */
    struct class *class;        /*  类          */
    struct device *device;      /*  设备        */
    struct device_node *nd;     /*  节点信息    */
    dev_t devid;                /*  设备号      */
    int major;                  /*  主设备号    */
    int minor;                  /*  次设备号    */
    int beep_gpio;              /*  beep所使用的GPIO编号 */
};

struct beep_dev beep;

static ssize_t beep_open(struct inode *inode, struct file *filp){
    filp->private_data = &beep;     /*  设置文件私有数据    */
    return 0;
}

static ssize_t beep_write(struct file *filp, const char __user *buf,
                            size_t cnt, loff_t *offt){
    int ret;
    unsigned char databuf[1];
    unsigned char beepstat;
    struct beep_dev *dev =  (struct beep_dev*)filp->private_data;

    ret = copy_from_user(databuf, buf, cnt);
    if(ret < 0){
        printk("kernel write failed\r\n");
        return -EFAULT;
    }

    beepstat = databuf[0];
    if(beepstat == BEEPON){
        gpio_set_value(dev->beep_gpio, 0);
    } else if(beepstat == BEEPOFF){
        gpio_set_value(dev->beep_gpio, 1);
    }

    return 0;
}

static ssize_t beep_release(struct inode *inode, struct file *filp){
    return 0;
}

static struct file_operations beep_fops = {
    .owner = THIS_MODULE,
    .open = beep_open,
    .write = beep_write,
    .release = beep_release,
};

static int __init beep_init(void){
    int ret = 0;

    /* 设置beep所使用的GPIO */
    /*  1.先获取节点信息    */
    beep.nd = of_find_node_by_path("/beep");
    if(beep.nd == NULL){
        printk("beep node can not found!\r\n");
        return -EINVAL;
    } else{
        printk("beep node has been found!\r\n");
    }

    /*  2.获取设备树中的GPIO属性，得到beep所使用的beep编号 */
    beep.beep_gpio = of_get_named_gpio(beep.nd, "beep-gpio", 0);
    if(beep.beep_gpio < 0){
        printk("can't get beep-gpio!\r\n");
        return -EINVAL;
    } else{
        printk("beep-gpio num = %d\r\n",beep.beep_gpio);
    }

    /*  3.申请一下GPIO管脚  */
    ret = gpio_request(beep.beep_gpio, "beep-gpio");
    if(ret < 0){
        printk("failed to request the beep-gpio\r\n");
        ret = -EINVAL;
        goto fail_fdnd;
    }

    /*  4.设置beep所使用的引脚（GPIO5_IO01）为输出，并且输出高电平
        默认关闭beep*/
    ret = gpio_direction_output(beep.beep_gpio, 1);
    if(ret < 0){
        printk("can't set gpio!\r\n");
        goto fail_setoutput;
    }

    /*  注册字符设备号  */
    if(beep.major){
        beep.devid = MKDEV(beep.major,0);
        ret = register_chrdev_region(beep.devid, BEEPCNT, BEEPNAME);
        
    } else{
        ret = alloc_chrdev_region(&beep.devid, 0, BEEPCNT, BEEPNAME);
        beep.major = MAJOR(beep.devid);
        beep.minor = MINOR(beep.devid);
    }

    if(ret < 0){
        goto fail_devid;
    }

    /*  添加字符设备    */
    beep.cdev.owner = THIS_MODULE;
    cdev_init(&beep.cdev, &beep_fops);
    ret = cdev_add(&beep.cdev, beep.devid, BEEPCNT);
    if(ret < 0){
        goto fail_cdevadd;
    }

    /*  自动添加设备节点    */
    beep.class = class_create(THIS_MODULE, BEEPNAME);
    if(IS_ERR(beep.class)){
        goto fail_classcrt;
    }

    beep.device = device_create(beep.class, NULL, beep.devid, NULL, BEEPNAME);
    if(IS_ERR(beep.device)){
        goto fail_devicecrt;
    }
    
    return 0;

fail_devicecrt:
    class_destroy(beep.class);
fail_classcrt:
    cdev_del(&beep.cdev);
fail_cdevadd:
    unregister_chrdev_region(beep.devid, BEEPCNT);
fail_devid:
fail_setoutput:
    gpio_free(beep.beep_gpio);
fail_fdnd:
    return ret;
}

static void __exit beep_exit(void){

    /*  删除字符设备    */
    cdev_del(&beep.cdev);

    /*  注销设备号  */
    unregister_chrdev_region(beep.devid, BEEPCNT);

    /*  注销设备节点    */
    device_destroy(beep.class, beep.devid);

    /*  删除类  */
    class_destroy(beep.class);
    /*  释放GPIO    */
    gpio_free(beep.beep_gpio);

}

/*  驱动注册和注销  */
module_init(beep_init);
module_exit(beep_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");