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


#define KEYINPUT_CNT        1
#define KEYINPUT_NAME       "keyinput"
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

struct keyinput_dev{
    
    struct device_node *nd;
    struct timer_list timer;
    struct irq_keydesc irqkeydesc[KEY_NUM];     /* 按键描述的数组 */
    unsigned char curkeynum;    /* 当前的按键号 */

    struct input_dev *inputdev;     /* 输入设备 */
};

struct keyinput_dev keyinputdev;

/* @description : 中断服务函数，开启定时器，延时 10ms，
 * 定时器用于按键消抖。
 * @param - irq : 中断号
 * @param - dev_id : 设备结构。
 * @return : 中断执行结果
*/
static irqreturn_t key0_handler(int irq, void *dev_id){
    /* 传进来的参数实际上是一个 struct keyinput_dev对象 */
    //int value = 0;
    struct keyinput_dev *dev = (struct keyinput_dev*)dev_id;
    
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
    struct keyinput_dev *dev = (struct keyinput_dev*)arg;

    num = dev->curkeynum;
    keydesc = &dev->irqkeydesc[num];

    value = gpio_get_value(keydesc->gpio);  /* 读取IO值 */
    if(value == 0){                         /* 按下按键 */
        /* 上报按键值 */
        //void input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
        input_event(dev->inputdev, EV_KEY, KEY_0, 1);
        //void input_sync(struct input_dev *dev)   上报完成之后要同步一次
        input_sync(dev->inputdev);
    } else{                                 /* 按键松开 */
        /* 上报按键值 */
        input_event(dev->inputdev, EV_KEY, KEY_0, 0);
        input_sync(dev->inputdev);
    }
}

/* 按键初始化 */
static int keyio_init(struct keyinput_dev *dev){
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
    dev->irqkeydesc[0].value = KEY_0;
    for(i=0; i<KEY_NUM; i++){
        ret = request_irq(dev->irqkeydesc[i].irqnum,
                            dev->irqkeydesc[i].handler,
                            IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                            dev->irqkeydesc[i].name,
                            &keyinputdev);
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



static int __init keyinput_init(void){
    int ret;
    struct keyinput_dev *dev = (struct keyinput_dev*)(&keyinputdev);
    /* 初始化IO */
    ret = keyio_init(dev);
    if(ret < 0){
        goto fail_keyinit;
    }

    /* 注册input_dev */
    keyinputdev.inputdev = input_allocate_device();
    if(keyinputdev.inputdev == NULL){
       ret = -ENOMEM;
       goto fail_keyinit;
    }

    /* keyinput的名称 */
    keyinputdev.inputdev->name = KEYINPUT_NAME;
#if 0
    set_bit(EV_KEY, dev->evbit);
	set_bit(KEY_POWER, dev->keybit);
#endif
    
    __set_bit(EV_KEY, keyinputdev.inputdev->evbit);     /* 设置按键产生事件 */
    __set_bit(EV_REP, keyinputdev.inputdev->evbit);     /* 设置重复事件 */
    __set_bit(KEY_0, keyinputdev.inputdev->keybit);      /* 设置产生哪些按键值，这个按键值不知道怎么确定的，\
                                                           但是正点原子上的按键对应key0 */
    ret = input_register_device(keyinputdev.inputdev);
    if(ret){
        goto fail_input_register;
    }

    return 0;
fail_input_register:
    input_free_device(keyinputdev.inputdev);
fail_keyinit:
    return ret;
}

static void __exit keyinput_exit(void){
    unsigned int i=0;
    /* 1.删除定时器 */
    del_timer(&keyinputdev.timer);

    /* 2.释放中断和IO */
    for(i=0; i<KEY_NUM; i++){
        free_irq(keyinputdev.irqkeydesc[i].irqnum, &keyinputdev);
        gpio_free(keyinputdev.irqkeydesc[i].gpio);
    }

    /* 3.注销input_dev */
    input_unregister_device(keyinputdev.inputdev);
    input_free_device(keyinputdev.inputdev);
}

module_init(keyinput_init);
module_exit(keyinput_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
