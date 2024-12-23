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
#define KEY_NUM             1

/* 中断IO描述结构体 */
struct irq_keydesc{
    int gpio;
    int irq_num;
    int value;
    char name[10];
    irqreturn_t (*handler)(int, void*);     /* 中断服务函数 */
};

struct keyinput_dev{
    struct device_node *nd;
    struct timer_list timer;
    struct irq_keydesc  irqkeydesc; 
    struct input_dev *inputdev;     /* 输入设备结构体 */
};

struct keyinput_dev keyinputdev;

/* 中断服务函数 */
static irqreturn_t key0_handler(int irq, void *dev_id){
    /* 传进来的参数实际上是一个 struct keyinput_dev对象 */ 
    struct keyinput_dev *dev = (struct keyinput_dev*)dev_id;
    dev->timer.data = (volatile long)dev;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(20));     /* 20ms的消抖 */

    return IRQ_RETVAL(IRQ_HANDLED);
}

/* 定时器函数 */
void timer_function(unsigned long arg){
    int value;
    struct irq_keydesc *keydesc;
    struct keyinput_dev *dev = (struct keyinput_dev*)arg;

    keydesc = &dev->irqkeydesc;
    value = gpio_get_value(keydesc->gpio);  /* 读取IO值 */
    if(value == 0){
        /* 上报按键值 */
        input_event(dev->inputdev, EV_KEY, BTN_0, 1);
        input_sync(dev->inputdev);
    } else{
        input_event(dev->inputdev, EV_KEY, BTN_0, 0);
        input_sync(dev->inputdev);
    }
}


/* 按键初始化*/
static int keyio_init(struct keyinput_dev *dev){

    int ret;
    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_findnd;
    }

    /* 找到对应的GPIO */
    dev->irqkeydesc.gpio = of_get_named_gpio(dev->nd,"key-gpio", 0);
    if(dev->irqkeydesc.gpio < 0){
        ret = -EINVAL;
        printk("can't get key0 gpio!\r\n");
        goto fail_gpio;

    }
    memset(dev->irqkeydesc.name, 0, sizeof(dev->irqkeydesc.name));
    sprintf(dev->irqkeydesc.name,"key%d", 0);
    ret = gpio_request(dev->irqkeydesc.gpio, dev->irqkeydesc.name);
    if(ret){
        ret = -EINVAL;
        goto fail_gpio;
    }

    /* 设置为输入模式  */
    gpio_direction_input(dev->irqkeydesc.gpio);
    
    dev->irqkeydesc.irqnum = irq_of_parse_and_map(dev->nd, 0); /* 获取中断号 */
    printk("key%d:gpio=%d, irqnum=%d\r\n", 0, dev->irqkeydesc.gpio, dev->irqkeydesc.irq_num);

    /* 申请中断 */
    dev->irqkeydesc.handler = key0_handler;
    dev->irqkeydesc.value = KEY_0;
    ret = request_irq(dev->irqkeydesc.irq_num,
                        dev->irqkeydesc.handler,
                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                        dev->irqkeydesc.name,
                        &keyinputdev);
    if(ret < 0){
        printk("irq %d request failed!\r\n",dev->irqkeydesc.irq_num);
        ret = -EINVAL;
        goto fail_irq;
    }

    /* 创建定时器 */
    init_timer(&dev->timer);
    dev->timer.function = timer_function;
    return 0;


fail_irq:
fail_gpio:
fail_findnd:
    return ret;
}

static int __init keyinput_init(void){
    int ret;
    struct keyinput_dev *dev = (struct keyinput_dev*)(&keyinputdev);
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

    __set_bit(EV_KEY, keyinputdev.inputdev->evbit);
    __set_bit(EV_REP, keyinputdev.inputdev->evbit);
    __set_bit(BTN_0, keyinputdev.inputdev->keybit);

    ret = input_register_device(keyinputdev.inputdev);
    if(ret) goto fail_input_register;
    return 0;

fail_input_register:
    input_free_device(keyinputdev.inputdev);
fail_keyinit:
    return ret;
}

static void __exit keyinput_exit(void){
    /* 1.删除定时器 */
    del_timer(&keyinputdev.timer);

    /* 2.释放中断和IO */
    free_irq(keyinputdev.irqkeydesc.irq_num, &keyinputdev);
    gpio_free(keyinputdev.irqkeydesc.gpio);

    /* 3.注销input_dev */
    input_unregister_device(keyinputdev.inputdev);
    input_free_device(keyinputdev.inputdev);
}

module_init(keyinput_init);
module_exit(keyinput_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");