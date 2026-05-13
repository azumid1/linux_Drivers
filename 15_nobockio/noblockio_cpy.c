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

#define IRQNAME     "noblockio"
#define IMX6ULL_CNT 1
#define KEY0VALUE   0x01
#define KEY_NUM     1
#define INVAKEY    0xff

struct irq_keydesc{
    int gpio;
    int irqnum;
    char name[10];
    unsigned char value;
    irqreturn_t (*handler)(int, void*); /* 中断处理函数 */
};

struct imx6ull_dev{
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    dev_t devid;
    int major;
    int minor;
    atomic_t keyvalue;
    atomic_t releasekey;    /* 标记是否完成一次按键 */
    unsigned char curkeynum;
    struct timer_list timer;
    struct irq_keydesc irqkey[KEY_NUM];
    wait_queue_head_t r_wait;
};

struct imx6ull_dev imx6ull;

/* 中断服务函数 */
static irqreturn_t key0_handler(int irq, void *dev_id){
    struct imx6ull_dev *dev = (struct imx6ull_dev *)dev_id;
    dev->curkeynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(20));

    return IRQ_RETVAL(IRQ_HANDLED);
}

void timer_func(unsigned long arg){
    unsigned char value;

    struct irq_keydesc *keydesc;
    struct imx6ull_dev *dev = (struct imx6ull_dev*)arg;
    keydesc = &dev->irqkey[dev->curkeynum];

    value = gpio_get_value(keydesc->gpio);
    if(value == 0)
        atomic_set(&dev->keyvalue, keydesc->value);
    else {
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);
    }

    /* 唤醒进程 */
    if(atomic_read(&dev->releasekey))
        wake_up(&dev->r_wait);
}

static int keyio_init(struct imx6ull_dev *dev){
    unsigned char i = 0;
    int ret = 0;
    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL) return -EINVAL;

    for(i=0; i<KEY_NUM; i++){
        dev->irqkey[i].gpio = of_get_named_gpio(dev->nd, "key-gpio", i);
        if(dev->irqkey[i].gpio < 0){
            printk("can't get key%d\r\n", i);
            return -EINVAL;
        }
        /* 初始化key所使用的IO，并设置为中断模式 */
        memset(dev->irqkey[i].name, 0, sizeof(dev->irqkey[i].name));
        sprintf(dev->irqkey[i].name, "key%d", i);

        ret = gpio_request(dev->irqkey[i].gpio, dev->irqkey[i].name);
        if(ret < 0){
            dev->curkeynum = i;
            goto fail_gpio2;

        }

        gpio_direction_input(dev->irqkey[i].gpio);
        dev->irqkey[i].irqnum = irq_of_parse_and_map(dev->nd, i);
        printk("key%d:gpio=%d, irqnum=%d\r\n",i,dev->irqkey[i].gpio, dev->irqkey[i].irqnum);
    }

    /* 申请中断 */
    imx6ull.irqkey[0].handler = key0_handler;
    imx6ull.irqkey[0].value = KEY0VALUE;
    
    ret = request_irq(imx6ull.irqkey[0].irqnum, imx6ull.irqkey[0].handler,
            IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, imx6ull.irqkey[0].name, &imx6ull);
    
    /* 创建定时器 */
    init_timer(&imx6ull.timer);
    imx6ull.timer.function = timer_func;

    /* 初始化等待队列头 */
    init_waitqueue_head(&imx6ull.r_wait);
    return 0;

fail_gpio2:
    for(i=0; i<dev->curkeynum; i++){
        gpio_free(dev->irqkey[i].gpio);
    }
    return -EINVAL;
}

static int imx6ull_open(struct inode *inode, struct file *filp){
    filp->private_data = &imx6ull;
    return 0;
}

static ssize_t imx6ull_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
    int ret = 0;
    unsigned char keyvalue = 0;
    unsigned char releasekey = 0;
    struct imx6ull_dev *dev = (struct imx6ull_dev*)filp->private_data;

    /* 非阻塞访问 */
    if(filp->f_flags & O_NONBLOCK){
        if(atomic_read(&dev->releasekey) == 0)
            return -EAGAIN;
    } else{
        /* 假如等待队列，等待被唤醒，也就是有按键按下 */
        ret = wait_event_interruptible(dev->r_wait, atomic_read(&dev->releasekey));
        if(ret) goto wait_error;
    }

    keyvalue = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);

    if(releasekey){
        if(keyvalue & 0x80){
            keyvalue &= ~0x80;
            ret = copy_to_user(buf, &keyvalue, sizeof(keyvalue));
        } else goto dataerr;
        atomic_set(&dev->releasekey, 0);
    }

    return 0;

wait_error:
    return ret;
dataerr:
    return -EINVAL;
}

unsigned int imx6ull_poll(struct file *filp, struct poll_table_struct *wait){
    unsigned int mask = 0;
    struct imx6ull_dev *dev = (struct imx6ull_dev*)filp->private_data;
    poll_wait(filp, &dev->r_wait, wait);
    if(atomic_read(&dev->releasekey)){
        mask = POLLIN | POLLRDNORM;
    }
    return mask;
}

static struct file_operatons imx6ull_fops = {
    .owner = THIS_MODULE,
    .open = imx6ull_open,
    .read = imx6ull_read,
    .poll = imx6ull_poll,
};

static int __init imx6ull_int(void){
    int ret;
    struct imx6ull_dev *dev = &imx6ull;
    if(dev->major){
        dev->devid = MKDEV(dev->major, 0);
        ret = register_chrdev_region(dev->devid, IMX6ULL_CNT, IRQNAME);
    } else{
        ret = alloc_chrdev_region(&dev->devid, 0, IMX6ULL_CNT, IRQNAME);
    }

    if(ret < 0) goto fail_def;


    cdev_init(&dev->cdev, &imx6ull_fops);
    ret = cdev_add(&dev->cdev, dev->devid, IMX6ULL_CNT);
    if(ret<0) goto fail_cdev;

    dev->class = class_create(THIS_MODULE, IRQNAME);
    if(IS_ERR(dev->class)) goto fail_class;

    dev->device = device_create(dev->class, NULL, dev->devid, NULL, IRQNAME);
    if(IS_ERR(dev->device)) goto fail_device;

    keyio_init();

fail_device:
    class_destroy(dev->class);
fail_class:
    cdev_del(&dev->cdev);
fail_cdev:
    unregister_chrdev_region(dev->devid, IMX6ULL_CNT);
fail_def:
    return ret;


}

static void __exit imx6ull_exit(void){
    /* 删除定时器 */
    del_timer(&imx6ull.timer);
    gpio_free(imx6ull.irqkey[0].gpio);
    free_irq(imx6ull.irqkey[0].irqnum, &imx6ull);
    device_destroy(imx6ull.class, imx6ull.devid);
    class_desatroy(imx6ull.calss);
    cdev_del(&imx6ull.cdev);
    unregister_chrdev_region(imx6ull.devid, IMX6ULL_CNT);
}

module_init(imx6ull_inti);
module_exit(imx6ull_exit);
MODULE_LIECNSE("GPL");
MOUDLE_AUTHOR("azumid");
