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

#define IMX6UIRQ_CNT    1
#define IMX6UIRQ_NAME   "imx6uirq"
#define KEY0VALUE       0x01
#define INVAKEY         0xFF
#define KEY_NUM         1

/* 中断IO描述结构体 */
struct irq_keydesc{
    int irqnum;
    int gpio;
    unsigned char value; /* gpio对应的值，是0还是1 */
    char name[10];
    irqreturn_t (*handler)(int, void*); /* 中断服务函数 */
};
/* 设备结构体 */
struct imx6uirq_dev{
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    dev_t devid;
    int major;
    int minor;
    atomic_t keyvalue;
    atomic_t releasekey;
    struct timer_list timer;
    struct irq_keydesc irqkey;
    struct fasync_struct *fasync_queue; /* 异步相关结构体 */
};

struct imx6uirq_dev imx6uirq;

void timer_func(unsigned long arg){
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)arg;
    struct irq_keydesc *keydesc = &dev->irqkey;

    if(gpio_get_value(keydesc->gpio) == 0){
        atomic_set(&dev->keyvalue, keydesc->value);
    } else{
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);    /* 完整的按键过程 */
    
    }

    if(atomic_read(&dev->releasekey)){  /* 有效的按键过程 */
        if(dev->fasync_queue){
            kill_fasync(&dev->fasync_queue, SIGIO, POLL_IN);    /* kill_fasync函数负责发送指定的信号 */
        }
    }
}

static irqreturn_t key0_handler(int irq, void *dev_id){
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)dev_id;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(20));

    return IRQ_RETVAL(IRQ_HANDLED);
}

static int keyio_init(struct imx6uirq_dev *dev){
    int ret;
    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_nd;
    }

    dev->irqkey.gpio = of_get_named_gpio(dev->nd, "key-gpio", 0);
    if(dev->irqkey.gpio < 0){
        printk("can't get key-gpio\r\n");
        goto fail_gpio;
    }

    /* 申请并设置gpio */
    memset(dev->irqkey.name, 0, sizeof(dev->irqkey.name));
    sprintf(dev->irqkey.name, "key%d", 0);
    ret = gpio_request(dev->irqkey.gpio, dev->irqkey.name);
    if(ret < 0){
        ret = -EINVAL;
        goto fail_gpio;
    }
    gpio_direction_input(dev->irqkey.gpio);

    /* 获取中断号 */
    dev->irqkey.irqnum = irq_of_parse_and_map(dev->nd, 0);
    printk("key0:gpio=%d, irqnum=%d\r\n",dev->irqkey.gpio,dev->irqkey.irqnum);

    dev->irqkey.handler = key0_handler;
    dev->irqkey.value = KEY0VALUE;

    ret = request_irq(dev->irqkey.irqnum,
                    dev->irqkey.handler,
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    dev->irqkey.name,
                    &imx6uirq);
    if(ret < 0){
        printk("irq %d request failed\r\n", dev->irqkey.irqnum);
        ret =  -EINVAL;
        goto fail_irq;  
    }

    /* 创建定时器 */
    init_timer(&dev->timer);
    dev->timer.function = timer_func;

    return 0;

fail_irq:
    gpio_free(dev->irqkey.gpio);
fail_gpio:
fail_nd:
    return ret;
}

static int imx6uirq_open(struct inode *inode, struct file *filp){
    filp->private_data = &imx6uirq;
    return 0;
}

static ssize_t imx6uirq_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
    int ret = 0;
    unsigned char keyvalue = 0;
    unsigned char releasekey = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)filp->private_data;
    keyvalue = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);

    if(releasekey){     /* 有键按下 */
        if(keyvalue & 0x80){
            keyvalue &= ~0x80;      /* 这一步是将最高位再次置为0 */
            ret = copy_to_user(buf, &keyvalue, sizeof(keyvalue));

        } else{
            goto dataerror;
        }

        atomic_set(&dev->releasekey, 0);    /* 按下标志位清零 */
    } else{
        goto dataerror;
    }

    return 0;

dataerror:
    return -EINVAL;

}

static int imx6uirq_fasync(int fd, struct file *filp, int on){
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)filp->private_data;
    return fasync_helper(fd, filp, on, &dev->fasync_queue);
}

static int imx6uirq_release(struct inode *inode, struct file *filp){
    return imx6uirq_fasync(-1, filp, 0);
}

static struct file_operations imx6uirq_fops = {
    .owner = THIS_MODULE,
    .open = imx6uirq_open,
    .read = imx6uirq_read,
    .fasync = imx6uirq_fasync,
    .release = imx6uirq_release,
};


static int __init imx6uirq_init(void){
    int ret = 0;
    struct imx6uirq_dev *dev = &imx6uirq;
    if(dev->major){
        dev->devid = MKDEV(dev->major, 0);
        ret = register_chrdev_region(dev->devid, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    } else{
        ret = alloc_chrdev_region(&dev->devid, 0,IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    }
    if(ret < 0) goto fail_def;

    cdev_init(&dev->cdev, &imx6uirq_fops);
    ret = cdev_add(&dev->cdev, dev->devid, IMX6UIRQ_CNT);
    if(ret < 0) goto fail_cdev;

    dev->class = class_create(THIS_MODULE, IMX6UIRQ_NAME);
    if(IS_ERR(dev->class)) goto fail_class;

    dev->device = device_create(dev->class, NULL, dev->devid, NULL, IMX6UIRQ_NAME);
    if(IS_ERR(dev->device)) goto fail_device;

    return 0;

    /* 初始化按键 */
    atomic_set(&dev->keyvalue, INVAKEY);
    atomic_set(&dev->releasekey, 0);
    keyio_init(dev);

fail_device:
    class_destroy(dev->class);
fail_class:
    cdev_del(&dev->cdev);
fail_cdev:
    unregister_chrdev_region(dev->devid, IMX6UIRQ_CNT);
fail_def:
    return ret;
}

static void __exit imx6uirq_exit(void){
    /* 删除定时器 */
    del_timer(&imx6uirq.timer);

    /* 释放中断和IO */
    free_irq(imx6uirq.irqkey.irqnum, &imx6uirq);
    gpio_free(imx6uirq.irqkey.gpio);


    cdev_del(&imx6uirq.cdev);
    unregister_chrdev_region(imx6uirq.devid, IMX6UIRQ_CNT);
    device_destroy(imx6uirq.class, imx6uirq.devid);
    class_destroy(imx6uirq.class);
}

module_init(imx6uirq_init);
module_exit(imx6uirq_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");