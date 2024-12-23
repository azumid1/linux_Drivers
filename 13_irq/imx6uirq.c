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

#define IMX6UIRQ_CNT        1
#define IMX6UIRQ_NAME       "imx6uirq"
#define KEY0VALUE           0x01
#define INVAKEY             0xFF
#define KEY_NUM             1           /* 按键的数量 */

/* 中断IO描述结构体 */
struct irq_keydesc{
    int gpio;
    int irqnum;             /* 中断号 */
    unsigned char value;    /* 按键对应的键值 */
    char name[10];
    irqreturn_t (*handler)(int, void*);     /* 中断服务函数 */
};

struct imx6uirq_dev{
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    dev_t devid;
    int major;
    int minor;
    atomic_t keyvalue;          /* 有效的按键值 */
    atomic_t releasekey;        /* 标记是否完成一次按键 */
    struct timer_list timer;
    struct irq_keydesc irqkeydesc[KEY_NUM];     /* 按键描述的数组 */
    unsigned char curkeynum;    /* 当前的按键号 */
};

struct imx6uirq_dev imx6uirq;

/* @description : 中断服务函数，开启定时器，延时 10ms，
 * 定时器用于按键消抖。
 * @param - irq : 中断号
 * @param - dev_id : 设备结构。
 * @return : 中断执行结果
*/
static irqreturn_t key0_handler(int irq, void *dev_id){
    /* 传进来的参数实际上是一个 struct imx6uirq_dev对象 */
    int value = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)dev_id;
    
    dev->curkeynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(20)); /* 进行20ms的消抖 */
    
    /* 下面这段代码是对中断处理函数的测试 */
    /*
    value = gpio_get_value(dev->irqkeydesc[0].gpio);
    if(value == 0){
        printk("KEY0 PUSH\r\n");
    } else if(value == 1){
        printk("KEY0 RELEASE\r\n");
    }
    */

    return IRQ_RETVAL(IRQ_HANDLED);

}

void timer_function(unsigned long arg){
    unsigned char value;
    unsigned char num;
    struct irq_keydesc *keydesc;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)arg;

    num = dev->curkeynum;
    keydesc = &dev->irqkeydesc[num];

    value = gpio_get_value(keydesc->gpio);  /* 读取IO值 */
    if(value == 0){                         /* 按下按键 */
        atomic_set(&dev->keyvalue, keydesc->value);
        //printk("KEY0 PUSH\r\n");

    } else{                                 /* 按键松开 */
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);
        //printk("KEY0 RELEASE\r\n");
    }
}

/* 按键初始化 */
static int keyio_init(struct imx6uirq_dev *dev){
    unsigned char i = 0;
    int ret = 0;

    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_def;
    }
    
    /* 提取GPIO */
    for(i=0; i<KEY_NUM;i++){
        dev->irqkeydesc[i].gpio = of_get_named_gpio(dev->nd, "key-gpio", i);
        if(dev->irqkeydesc[i].gpio < 0){
            printk("can't get key%d\r\n",i);
        }
    }

    /* 初始化key所使用的IO，并设置为中断模式 */
    for(i=0; i<KEY_NUM; i++){
        memset(dev->irqkeydesc[i].name, 0, sizeof(dev->irqkeydesc[i].name));
        sprintf(dev->irqkeydesc[i].name, "key%d",i);
        ret = gpio_request(dev->irqkeydesc->gpio, dev->irqkeydesc[i].name);
        if(ret){
            ret =  -EINVAL;
            goto fail_def;
        }
        gpio_direction_input(dev->irqkeydesc[i].gpio);
        dev->irqkeydesc[i].irqnum = irq_of_parse_and_map(dev->nd, i); /* 获取中断号 */
#if 0
    dev->irqkeydesc[i].irqnum = gpio_to_irq(dev->irqkeydesc[i].gpio);
#endif
        printk("key%d:gpio=%d, irqnum=%d\r\n",i,dev->irqkeydesc[i].gpio,dev->irqkeydesc[i].irqnum);

    }
    /* 申请中断 */
    dev->irqkeydesc[0].handler = key0_handler;
    dev->irqkeydesc[0].value = KEY0VALUE;
    for(i=0; i<KEY_NUM; i++){
        ret = request_irq(dev->irqkeydesc[i].irqnum,
                            dev->irqkeydesc[i].handler,
                            IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                            dev->irqkeydesc[i].name,
                            &imx6uirq);
        if(ret < 0){
            printk("irq %d request failed\r\n", dev->irqkeydesc[i].irqnum);
            ret =  -EINVAL;
            goto fail_irq;

        }
    }

    /* 创建定时器 */
    init_timer(&dev->timer);
    dev->timer.function = timer_function;
    return 0;


fail_irq:
    for(i=0; i<KEY_NUM; i++){
        gpio_free(dev->irqkeydesc[i].gpio);
    }
fail_def:
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

/* 设备操作函数 */
static struct file_operations imx6uirq_fops = {
    .owner = THIS_MODULE,
    .open = imx6uirq_open,
    .read = imx6uirq_read,
};

static int __init imx6uirq_init(void){
    int ret;
    struct imx6uirq_dev *dev = &imx6uirq;
    if(dev->major){
        dev->devid = MKDEV(dev->major,0);
        ret = register_chrdev_region(dev->devid, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    } else{
        ret = alloc_chrdev_region(&dev->devid, 0, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
        dev->major = MAJOR(dev->devid);
        dev->minor = MINOR(dev->devid);
    }
    if(ret < 0){
        goto fail_def;
    }

    cdev_init(&dev->cdev, &imx6uirq_fops);
    ret = cdev_add(&dev->cdev, dev->devid, IMX6UIRQ_CNT);
    if(ret < 0){
        goto fail_cdev;
    }
    dev->class = class_create(THIS_MODULE, IMX6UIRQ_NAME);
    if(IS_ERR(dev->class)){
        // return PTR_ERR(dev->class);
        goto fail_class;
    }

    dev->device = device_create(dev->class, NULL, dev->devid, NULL, IMX6UIRQ_NAME);

    if(IS_ERR(dev->device)){
        // return PTR_ERR(dev->device);
        goto fail_device;
    }

    /* 初始化按键 */
    atomic_set(&dev->keyvalue, INVAKEY);
    atomic_set(&dev->releasekey, 0);
    keyio_init(dev);

    return 0;

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
    unsigned int i=0;
    /* 删除定时器 */
    del_timer(&imx6uirq.timer);

    /* 释放中断和IO */
    for(i=0; i<KEY_NUM; i++){
        free_irq(imx6uirq.irqkeydesc[i].irqnum, &imx6uirq);
        gpio_free(imx6uirq.irqkeydesc[i].gpio);
    }

    cdev_del(&imx6uirq.cdev);
    unregister_chrdev_region(imx6uirq.devid, IMX6UIRQ_CNT);
    device_destroy(imx6uirq.class, imx6uirq.devid);
    class_destroy(imx6uirq.class);
}

module_init(imx6uirq_init);
module_exit(imx6uirq_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
