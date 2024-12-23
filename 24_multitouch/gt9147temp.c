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
#include<linux/i2c.h>
#include<linux/delay.h>

#define GT9147_CNT     1
#define GT9147_NAME    "gt9147"

struct gt9147_dev{
    int major;
    int minor;
    dev_t devid;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    void *private_data;
    unsigned short ir, als, ps;

};

static struct gt9147_dev gt9147dev;

/* 读取AP3216C的N个寄存器值 */
static int gt9147_read_regs(struct gt9147_dev *dev, u8 reg, void *val, int len){
    
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client*)dev->private_data;
    
    /* msg[0]发送要读取的寄存器首地址 */
    msg[0].addr = client->addr;     /* 从机地址，也就是ap3216c的地址 */
    msg[0].flags = 0;               /* 表示要发送数据 */
    msg[0].buf = &reg;              /* 要发送的数据，也就是寄存器的地址 */
    msg[0].len = 1;                 /* 发送的数据长度为1字节 */

    /* msg[1]读取数据 */
    msg[1].addr = client->addr;     /* 从机地址，也就是ap3216c的地址 */
    msg[1].flags = I2C_M_RD;        /* 表述读数据 */
    msg[1].buf = val;               /* 接收到从机发送的数据,就是将读取到的数据保存到val中 */
    msg[1].len = len;               /* 要读取的寄存器的长度 */
    return i2c_transfer(client->adapter, msg, 2);
}

/* 写入AP3216C的N个寄存器值 */
static int gt9147_write_regs(struct gt9147_dev *dev, u8 reg, u8 *buf, u8 len){
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client*)dev->private_data;
    /* 构建要发送的数据，也就是寄存器首地址+实际的数据 */
    b[0] = reg;
    memcpy(&b[1], buf, len);

    /* msg发送要读取的寄存器首地址 */
    msg.addr = client->addr;     /* 从机地址，也就是ap3216c的地址 */
    msg.flags = 0;               /* 表示要发送数据 */
    msg.buf = b;                 /* 要发送的数据，也就是寄存器的地址+实际的数据 */
    msg.len = len + 1;           /* 发送的数据长度为寄存器地址长度+实际数据的长度 */

    return i2c_transfer(client->adapter, &msg, 1);

}

/* 向gt9147一个寄存器写数据 */
static void gt9147_write_reg(struct gt9147_dev *dev, u8 reg, u8 data){
    u8 buf = 0;
    buf = data;
    gt9147_write_regs(dev, reg, &buf, 1);

}

/* 传统的匹配表 */
static struct i2c_device_id gt9147_id[] = {
    {"goodix,gt9147",0},
    {}
};

/* 设备树匹配表 */
static struct of_device_id gt9147_of_match[] = {
    {.compatible = "goodix,gt9147"},
    {}
};

static int gt9147_open(struct inode *inode, struct file *filp){
    unsigned char value = 0;
    filp->private_data = &gt9147dev;       /*  这是设置了文件私有数据  */
    
    
    return 0;
}

static ssize_t gt9147_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
   
    return 0;
}


static int gt9147_release(struct inode *inode, struct file *filp){
    //struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;    /*  这是提取了私有数据  */
   
    return 0;
}

static struct file_operations gt9147_fops = {
    .owner = THIS_MODULE,
    .open =gt9147_open,
    .read = gt9147_read,
    .release = gt9147_release,
};


static int gt9147_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    int ret = 0;
    
    /* 搭建字符设备框架 */
    /* 1、创建设备号 */
	if (gt9147dev.major) {		/*  定义了设备号 */
		gt9147dev.devid = MKDEV(gt9147dev.major, 0);
		ret = register_chrdev_region(gt9147dev.devid, AP3216C_CNT, AP3216C_NAME);
	} else {						/* 没有定义设备号 */
		ret = alloc_chrdev_region(&gt9147dev.devid, 0, AP3216C_CNT, AP3216C_NAME);	/* 申请设备号 */
		gt9147dev.major = MAJOR(gt9147dev.devid);	/* 获取分配号的主设备号 */
		gt9147dev.minor = MINOR(gt9147dev.devid);	/* 获取分配号的次设备号 */
	}
    if(ret < 0){
        goto fail_devid;
    }
	printk("gt9147 major=%d,minor=%d\r\n",gt9147dev.major, gt9147dev.minor);	
	
	/* 2、初始化cdev */
	gt9147dev.cdev.owner = THIS_MODULE;
	cdev_init(&gt9147dev.cdev, &gt9147_fops);
	
	/* 3、添加一个cdev */
	ret = cdev_add(&gt9147dev.cdev, gt9147dev.devid, GT9147_CNT);
    if(ret < 0){
        goto fail_cdev;
    } 
	/* 4、创建类 */
	gt9147dev.class = class_create(THIS_MODULE, GT9147_NAME);
	if(IS_ERR(gt9147dev.class)){
        ret = PTR_ERR(gt9147dev.class);
        goto fail_classcrt;
    }

	/* 5、创建设备 */
	gt9147dev.device = device_create(gt9147dev.class, NULL, gt9147dev.devid, NULL, GT9147_NAME);
	if(IS_ERR(gt9147dev.device)){
        ret = PTR_ERR(gt9147dev.device);
        goto fail_devicecrt;
    }

    gt9147dev.private_data = client;

    return 0;
fail_devicecrt:
    class_destroy(gt9147dev.class);
fail_classcrt:
    cdev_del(&gt9147dev.cdev);
fail_cdev:
    unregister_chrdev_region(gt9147dev.devid, AP3216C_CNT);
fail_devid:
    return ret;

}

static int gt9147_i2c_remove(struct i2c_client *i2c){
    
    cdev_del(&gt9147dev.cdev);
    unregister_chrdev_region(gt9147dev.devid, GT9147_CNT);

    device_destroy(gt9147dev.class, gt9147dev.devid);
    class_destroy(gt9147dev.class);
   
    return 0;
}


/* i2c_driver */
static struct i2c_driver gt9147_driver = {
    .probe = gt9147_i2c_probe,
    .remove = gt9147_i2c_remove,
    .driver = {
        .name = "gt9147",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(gt9147_of_match),
    },
    .id_table = gt9147_id,
};


static int __init gt9147_init(void){
    int ret = 0;
    ret = i2c_add_driver(&gt9147_driver);
    return ret;
}

static void __exit gt9147_exit(void){
    i2c_del_driver(&gt9147_driver);
}

module_init(gt9147_init);
module_exit(gt9147_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
