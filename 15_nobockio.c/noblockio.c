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
#include<linux/wait.h>
#include<linux/poll.h>
#define IMX6UIRQ_CNT        1
#define IMX6UIRQ_NAME       "noblockio"
#define KEY0VALUE           0x01
#define INVAKEY             0xFF
#define KEY_NUM             1           /* 按键的数量 */

struct irq_keydesc{
    int gpio;
    int irqnum;
    char name[10];
    unsigned char value;    /* 按键当前对应的值 */
    irqreturn_t (*handler)(int, void*); /* 中断处理函数 */
};

struct imx6uirq_dev{
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    dev_t devid;
    int major;
    int minor;
    atomic_t keyvalue;      /* 有效的按键值 */
    atomic_t releasekey;    /* 标记是否完成一次按键 */
    unsigned char curkeynum;    /* 当前的按键号 */
    struct timer_list timer;
    struct irq_keydesc irqkey[KEY_NUM];
    wait_queue_head_t r_wait;   /* 读等待队列头 */
};

struct imx6uirq_dev imx6uirq;

/* 中断服务函数 */
static irqreturn_t key0_handler(int irq, void *dev_id){
    
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)dev_id;
    dev->curkeynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(20));

    return IRQ_RETVAL(IRQ_HANDLED);
}

void timer_func(unsigned long arg){
    unsigned char value;

    struct irq_keydesc *keydesc;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)arg;
    keydesc = &dev->irqkey[dev->curkeynum];

    value = gpio_get_value(keydesc->gpio);
    if(value == 0){
        atomic_set(&dev->keyvalue, keydesc->value);
    } else{
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);
    }

    /* 唤醒进程 */
    if(atomic_read(&dev->releasekey)){
        wake_up(&dev->r_wait);
    }
}

static int keyio_init(struct imx6uirq_dev *dev){
    unsigned char i = 0;
    int ret = 0;
    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_nd;
    }

    for(i=0; i<KEY_NUM; i++){
        dev->irqkey[i].gpio = of_get_named_gpio(dev->nd, "key-gpio", i);
        if(dev->irqkey[i].gpio < 0){
            printk("can't get key%d\r\n",i);
            goto fail_gpio;
        }

        /* 初始化key所使用的IO， 并设置为中断模式 */
        memset(dev->irqkey[i].name, 0, sizeof(dev->irqkey[i].name));
        sprintf(dev->irqkey[i].name, "key%d", i);
        ret = gpio_request(dev->irqkey[i].gpio,dev->irqkey[i].name);
        if(ret < 0){
            dev->curkeynum = i;
            goto fail_gpio2;
        }
        gpio_direction_input(dev->irqkey[i].gpio);
        dev->irqkey[i].irqnum = irq_of_parse_and_map(dev->nd, i);
        printk("key%d:gpio=%d, irqnum=%d\r\n",i,dev->irqkey[i].gpio, dev->irqkey[i].irqnum);
   
    }

    /* 申请中断 */
    dev->irqkey[0].handler = key0_handler;
    dev->irqkey[0].value = KEY0VALUE;
    ret = request_irq(dev->irqkey[0].irqnum,
                    dev->irqkey[0].handler,
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    dev->irqkey[0].name,
                    &imx6uirq);
    
    if(ret < 0){
        printk("irq %d request failed\r\n", dev->irqkey[0].irqnum);
        ret =  -EINVAL;
        goto fail_irq;
    }

    /* 创建定时器 */
    init_timer(&dev->timer);
    dev->timer.function = timer_func;

    /* 初始化等待队列头 */
    init_waitqueue_head(&dev->r_wait);

    return 0;
fail_irq:
fail_gpio2:
    for(i=0; i<dev->curkeynum; i++){
        gpio_free(dev->irqkey[i].gpio);
    }
    
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
    unsigned char releasekey  = 0;
    struct imx6uirq_dev  *dev = (struct imx6uirq_dev*)filp->private_data;

    if(filp->f_flags & O_NONBLOCK){
        if(atomic_read(&dev->releasekey) == 0)
            return -EAGAIN;
    } else{
        /* 加入等待队列，等待被唤醒， 也就是有按键按下 */
        ret = wait_event_interruptible(dev->r_wait, atomic_read(&dev->releasekey));
        if(ret) goto wait_error;
    }

    keyvalue = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);
    
    if (releasekey){
        if(keyvalue & 0x80){
            keyvalue &= ~0x80;
            ret = copy_to_user(buf, &keyvalue, sizeof(keyvalue)); 
        } else{
            goto dataerr;
        }
        atomic_set(&dev->releasekey, 0);    /* 将按下标志清零 */
    }else goto dataerr;  

    return 0;
wait_error:
    return ret;
dataerr:
    return -EINVAL;

}

unsigned int imx6uirq_poll(struct file *filp, struct poll_table_struct *wait){
    unsigned int mask = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)filp->private_data;
    poll_wait(filp, &dev->r_wait, wait);
    if(atomic_read(&dev->releasekey)){
        mask = POLLIN | POLLRDNORM;
    }
    return mask;
}

static struct file_operations imx6uirq_fops = {
    .owner = THIS_MODULE,
    .open = imx6uirq_open,
    .read = imx6uirq_read,
    .poll = imx6uirq_poll,
};

static int __init imx6uirq_init(void){
    int ret;
    struct imx6uirq_dev *dev = &imx6uirq;
    if(dev->major){
        dev->devid = MKDEV(dev->major, 0);
        ret = register_chrdev_region(dev->devid, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    } else{
        ret = alloc_chrdev_region(&dev->devid, 0, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    }
    if(ret < 0) goto fail_def;

    cdev_init(&dev->cdev, &imx6uirq_fops);
    ret = cdev_add(&dev->cdev, dev->devid, IMX6UIRQ_CNT);
    if(ret < 0) goto fail_cdev;

    dev->class = class_create(THIS_MODULE, IMX6UIRQ_NAME);
    if(IS_ERR(dev->class)) goto fail_class;

    dev->device = device_create(dev->class, NULL, dev->devid, NULL, IMX6UIRQ_NAME);
    if(IS_ERR(dev->device)) goto fail_device;

    keyio_init(&imx6uirq);

    /* 初始化按键 */
    atomic_set(&dev->keyvalue, INVAKEY);
    atomic_set(&dev->releasekey, 0);

    /* 等待队列头 */
    init_waitqueue_head(&imx6uirq.r_wait);

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
    unsigned int i = 0;

    /* 删除定时器 */
    del_timer(&imx6uirq.timer);
    /* 释放中断和IO */
    for(i=0; i<KEY_NUM; i++){
        gpio_free(imx6uirq.irqkey[i].gpio);
        free_irq(imx6uirq.irqkey[i].irqnum, &imx6uirq);
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