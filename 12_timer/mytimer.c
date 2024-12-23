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
#include<linux/timer.h>

#define TIMER_NAME      "timer"
#define TIMER_CNT        1
#define LEDOFF           0
#define LEDON            1
#define CLOSE_CMD        (_IO(0xEF, 0x1))   /* 关闭定时器 */
#define OPEN_CMD         (_IO(0xEF, 0x2))   /* 打开定时器 */
#define SETPERIOD_CMD    (_IO(0xEF, 0x3))   /* 设置定时器周期命令 */

/* timer结构体 */
struct mytimer_dev{
    struct cdev cdev;           /* 字符设备     */
    struct class *class;        /* 类           */
    struct device *device;      /* 设备         */
    dev_t devid;                /* 设备号       */
    int major;                  /* 主设备号     */
    int minor;                  /* 次设备号     */
    struct device_node *nd;     /* 节点信息     */
    int led_gpio;               /*  led所使用的GPIO编号 */
    int timerperiod;            /* 定时周期，单位为 ms */
    struct timer_list timer;    /* 定义一个定时器 */
    spinlock_t lock;            /* 定义自旋锁 */
};

struct mytimer_dev mytimer;

static int led_init(struct mytimer_dev *dev){
    int ret = 0;
    dev->nd = of_find_node_by_path("/gpioled");
    if(dev->nd == NULL){
        return -EINVAL;
    }

    dev->led_gpio = of_get_named_gpio(dev->nd, "led-gpio", 0);
    if(dev->led_gpio < 0){
        printk("can't get led\r\n");
        return -EINVAL;
    }

    /* 初始化 led 所使用的 IO */
    gpio_request(dev->led_gpio, "led");
    ret = gpio_direction_output(dev->led_gpio, 1);
    if(ret < 0){
        printk("can't set gpio\r\n");
        gpio_free(mytimer.led_gpio);
        return ret;
    }

    return 0;
}   


static int mytimer_open(struct inode *inode, struct file *filp){
    filp->private_data = &mytimer;       /*  这是设置了文件私有数据  */
    return 0;
}

static long timer_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
    struct mytimer_dev *dev = (struct mytimer_dev*)filp->private_data;
    int timerperiod;
    unsigned long flags;

    switch(cmd){
        case CLOSE_CMD:
            del_timer_sync(&dev->timer);
            break;
        case OPEN_CMD:
            spin_lock_irqsave(&dev->lock, flags);
            timerperiod = dev->timerperiod;
            spin_unlock_irqrestore(&dev->lock, flags);
            mod_timer(&dev->timer, jiffies + msecs_to_jiffies(timerperiod));
            break;
        case SETPERIOD_CMD:
            spin_lock_irqsave(&dev->lock, flags);
            dev->timerperiod = arg;
            spin_unlock_irqrestore(&dev->lock, flags);
            mod_timer(&dev->timer, jiffies + msecs_to_jiffies(dev->timerperiod));
            break;
        default:
            break;            
    }

    return 0;

}

static int mytimer_release(struct inode *inode, struct file *filp){
    //struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;    /*  这是提取了私有数据  */
    return 0;
}

static struct file_operations mytimer_fops = {
    .owner = THIS_MODULE,
    .open = mytimer_open,
    .unlocked_ioctl = timer_unlocked_ioctl,
    .release = mytimer_release,
};

/* 定时器回调函数 */
void timer_function(unsigned long arg){
    struct mytimer_dev *dev = (struct mytimer_dev*)arg;     /* 这里传过来的值是地址，然后又转换成 struct mytimer_dev 类型的 牛逼*/
    static int sta = 1;
    int timerperiod;
    unsigned long flags;

    sta = !sta;         /* 每次都取反，实现LED灯反转 */

    gpio_set_value(dev->led_gpio, sta);

    /* 重启定时器 */
    spin_lock_irqsave(&dev->lock, flags);
    timerperiod = dev->timerperiod;
    spin_unlock_irqrestore(&dev->lock, flags);
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(dev->timerperiod));   /* mod_timer会重新开启定时器 */
}

static int __init mytimer_init(void)
{
    int ret;
    mytimer.nd = NULL;
    mytimer.timerperiod = 1000; /* 默认周期为 1s */

    led_init(&mytimer);

    /* 初始化自旋锁 */
    spin_lock_init(&mytimer.lock);

    /* 注册新字符设备号 */
    mytimer.major = 0;       /* 设备号由内核分配 */
    if(mytimer.major){       /* 设备号已经给定   */
        mytimer.devid = MKDEV(mytimer.major, 0);
        ret = register_chrdev_region(mytimer.devid, TIMER_CNT, TIMER_NAME);
    } else {                /* 没有给定设备号   */
        ret = alloc_chrdev_region(&mytimer.devid, 0, TIMER_CNT, TIMER_NAME);
        mytimer.major = MAJOR(mytimer.devid);
        mytimer.minor = MINOR(mytimer.devid);
    }

    if(ret < 0){
        goto fail_devid;
    }

    /* 添加字符设备 */
    mytimer.cdev.owner = THIS_MODULE;
    cdev_init(&mytimer.cdev, &mytimer_fops);
    ret = cdev_add(&mytimer.cdev, mytimer.devid, TIMER_CNT);
    if(ret < 0){
        goto fail_cdevadd;
    } 

    /* 自动创建设备节点 */
    mytimer.class = class_create(THIS_MODULE, TIMER_NAME);
    if(IS_ERR(mytimer.class)){
        ret = PTR_ERR(mytimer.class);
        goto fail_classcrt;
    }

    mytimer.device = device_create(mytimer.class, NULL, mytimer.devid, NULL, TIMER_NAME);
    if(IS_ERR(mytimer.device)){
        ret = PTR_ERR(mytimer.device);
        goto fail_devicecrt;
    }

    /* 初始化timer, 设置定时器处理函数，还未设置周期，所以不会激活定时器 */
    init_timer(&mytimer.timer);
    mytimer.timer.function = timer_function;
    mytimer.timer.data = (unsigned long)&mytimer;   /* 这个其实是把地址传给了timer_function函数 ，很精彩*/
    //add_timer(&mytimer.timer);
    return 0;

fail_devicecrt:
    class_destroy(mytimer.class);
fail_classcrt:
    cdev_del(&mytimer.cdev);
fail_cdevadd:
    unregister_chrdev_region(mytimer.devid, TIMER_CNT);
fail_devid:
    return ret;
}

static void __exit mytimer_exit(void)
{
    gpio_set_value(mytimer.led_gpio, 1);   /* 卸载驱动时关闭LED */
    del_timer_sync(&mytimer.timer);        /* 删除 timer */
#if 0
    del_timer(&mytimer.timer);
#endif
    
    /* 删除字符设备 */
    cdev_del(&mytimer.cdev);

    /* 删除设备号   */
    unregister_chrdev_region(mytimer.devid,TIMER_CNT);

    /* 注销设备节点 */
    device_destroy(mytimer.class, mytimer.devid);

    /* 删除类       */
    class_destroy(mytimer.class);

    /* 释放IO */
    gpio_free(mytimer.led_gpio);
}

/* 注册和卸载驱动 */
module_init(mytimer_init);
module_exit(mytimer_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");