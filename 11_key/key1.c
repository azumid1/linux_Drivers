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

#define KEYNAME            "key"
#define KEYCNT              1
#define KEY0VALUE           0xF0    /*  按键值  */  
#define INVAKEY             0x00    /*  无效的按键值    */

struct key_dev{
    struct cdev cdev;           /*  字符设备    */
    struct class *class;        /*  类          */
    struct device *device;      /*  设备        */
    struct device_node *nd;     /*  节点信息    */
    dev_t devid;                /*  设备号      */
    int major;                  /*  主设备号    */
    int minor;                  /*  次设备号    */
    int key_gpio;              /*  beep所使用的GPIO编号 */
    atomic_t keyvalue;          /*  按键值  */
};

struct key_dev key;

static int keyio_init(struct key_dev *dev){
    int ret = 0;
    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        return -EINVAL;
    }

    dev->key_gpio = of_get_named_gpio(dev->nd, "key-gpio", 0);
    if(dev->key_gpio < 0){
        printk("can't get key0\r\n");
        return -EINVAL;
    }

    printk("key-gpio = %d\r\n",dev->key_gpio);

    /* 初始化key所使用的IO */
    ret = gpio_request(dev->key_gpio,"key");
    if(ret){
        return -EBUSY;
    }
    ret = gpio_direction_input(dev->key_gpio);
    if(ret < 0){
        gpio_free(dev->key_gpio);
    }
    return 0;
}

static int key_open(struct inode *inode, struct file *filp){
    
    int ret = 0;
    filp->private_data = &key;     /*  设置文件私有数据    */
    
    // ret = keyio_init(filp->private_data);
    // if(ret < 0){
    //     return ret;
    // }
    return 0;
}

static ssize_t key_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
    int ret = 0;
    int value;
    struct key_dev *dev = (struct key_dev*)filp->private_data;

    if(gpio_get_value(dev->key_gpio) == 0){ /* key0按下 */
        while(!gpio_get_value(dev->key_gpio)); /* 等待按键释放 */
        atomic_set(&dev->keyvalue, KEY0VALUE);
    } else{   /* 无效的按键值 */
        atomic_set(&dev->keyvalue,INVAKEY);
    }

    value = atomic_read(&dev->keyvalue);
    ret = copy_to_user(buf, &value, sizeof(value));
    return ret;
}

static ssize_t key_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt){
    
    return 0;
}

static int key_release(struct inode *inode, struct file *filp){
    return 0;
}

static struct file_operations key_fops = {
    .owner = THIS_MODULE,
    .open = key_open,
    .write = key_write,
    .read = key_read,
    .release = key_release,
};

static int __init mykey_init(void){
    int ret = 0;
    
   /* 初始化原子变量 */
   atomic_set(&key.keyvalue, INVAKEY);
  
    /*  注册字符设备号  */
    if(key.major){
        key.devid = MKDEV(key.major,0);
        ret = register_chrdev_region(key.devid, KEYCNT, KEYNAME);
        
    } else{
        ret = alloc_chrdev_region(&key.devid, 0, KEYCNT, KEYNAME);
        key.major = MAJOR(key.devid);
        key.minor = MINOR(key.devid);
    }

    if(ret < 0){
        goto fail_devid;
    }

    /*  添加字符设备    */
    key.cdev.owner = THIS_MODULE;
    cdev_init(&key.cdev, &key_fops);
    ret = cdev_add(&key.cdev, key.devid, KEYCNT);
    if(ret < 0){
        goto fail_cdevadd;
    }

    /*  自动添加设备节点    */
    key.class = class_create(THIS_MODULE, KEYNAME);
    if(IS_ERR(key.class)){
        goto fail_classcrt;
    }

    key.device = device_create(key.class, NULL, key.devid, NULL, KEYNAME);
    if(IS_ERR(key.device)){
        goto fail_devicecrt;
    }

    keyio_init(&key);
    
    return 0;

fail_devicecrt:
    class_destroy(key.class);
fail_classcrt:
    cdev_del(&key.cdev);
fail_cdevadd:
    unregister_chrdev_region(key.devid, KEYCNT);
fail_devid:

    return ret;
}

static void __exit mykey_exit(void){

    /*  删除字符设备    */
    cdev_del(&key.cdev);

    /*  注销设备号  */
    unregister_chrdev_region(key.devid, KEYCNT);

    /*  注销设备节点    */
    device_destroy(key.class, key.devid);

    /*  删除类  */
    class_destroy(key.class);
    
    /* 注销GPIO */
    gpio_free(key.key_gpio);
}

/*  驱动注册和注销  */
module_init(mykey_init);
module_exit(mykey_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");