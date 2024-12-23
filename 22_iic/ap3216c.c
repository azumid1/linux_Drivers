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
#include "ap3216creg.h"
#include<linux/delay.h>

#define AP3216C_CNT     1
#define AP3216C_NAME    "ap3216c"

struct ap3216c_dev{
    int major;
    int minor;
    dev_t devid;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    void *private_data;
    unsigned short ir, als, ps;

};

static struct ap3216c_dev ap3216cdev;

/* 读取AP3216C的N个寄存器值 */
static int ap3216c_read_regs(struct ap3216c_dev *dev, u8 reg, void *val, int len){
    
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
static int ap3216c_write_regs(struct ap3216c_dev *dev, u8 reg, u8 *buf, u8 len){
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

/* 读取AP3216C一个寄存器 */
static unsigned char ap3216c_read_reg(struct ap3216c_dev *dev, u8 reg){
    u8 data = 0;
    ap3216c_read_regs(dev, reg, &data, 1);
    return data;
}

/* AP3216C数据读取 */
void ap3216c_readdata(struct ap3216c_dev *dev)
{
    unsigned char buf[6];
    unsigned char i;

	/* 循环读取所有传感器数据 */
    for(i = 0; i < 6; i++)	
    {
        buf[i] = ap3216c_read_reg(dev, AP3216C_IRDATALOW + i);
        // buf[i] = ap3216c_readonebyte(AP3216C_ADDR, AP3216C_IRDATALOW + i);	
    }
	
    if(buf[0] & 0X80){  /* 为真表示IR和PS数据无效 */
        dev->ir = 0;
        dev->ps = 0;
    } 						
	else 				/* 读取IR传感器的数据   		*/
    {
        /**
         * 详细解释见驱动开发手记！！！
         * */ 
        dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0x03);  
        dev->ps = (((unsigned short)buf[5] & 0x3F) << 4) | (buf[4] & 0x0F);
    }
    dev->als = ((unsigned short)buf[3] << 8) | buf[2];
}

/* 向AP3216C一个寄存器写数据 */
static void ap3216c_write_reg(struct ap3216c_dev *dev, u8 reg, u8 data){
    u8 buf = 0;
    buf = data;
    ap3216c_write_regs(dev, reg, &buf, 1);

}


/* 传统的匹配表 */
static struct i2c_device_id ap3216c_id[] = {
    {"alientek,ap3216c",0},
    {}
};

/* 设备树匹配表 */
static struct of_device_id ap3216c_of_match[] = {
    {.compatible = "alientek,ap3216c"},
    {}
};

static int ap3216c_open(struct inode *inode, struct file *filp){
    unsigned char value = 0;
    filp->private_data = &ap3216cdev;       /*  这是设置了文件私有数据  */
    
    printk("ap3216c_open!\r\n");

    /* 初始化AP3216C */
    ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0X04);       /* 复位AP3216C 			*/
    mdelay(50);													    /* AP33216C复位至少10ms */
    ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0X03);       /* 开启ALS、PS+IR 		   	*/
    value = ap3216c_read_reg(&ap3216cdev, AP3216C_SYSTEMCONG);               /* 读取刚刚写进去的0X03 */
    printk("AP3216C_SYSTEMCONG = %#x\r\n", value);
    return 0;
}

static ssize_t ap3216c_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    long err = 0;
    short data[3];
    struct ap3216c_dev *dev = (struct ap3216c_dev*)filp->private_data;
    printk("ap3216c_read!\r\n");
    /* 向应用返回AP3216C的原始数据 */
    ap3216c_readdata(dev);

    data[0] = dev->ir;
    data[1] = dev->als;
    data[2] = dev->ps;  

    err = copy_to_user(buf, data, sizeof(data)); 
    return 0;
}


static int ap3216c_release(struct inode *inode, struct file *filp){
    //struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;    /*  这是提取了私有数据  */
    printk("ap3216c_release!\r\n");
    return 0;
}

static struct file_operations ap3216c_fops = {
    .owner = THIS_MODULE,
    .open = ap3216c_open,
    .read = ap3216c_read,
    .release = ap3216c_release,
};


static int ap3216c_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    int ret = 0;
    printk("ap3216c_i2c_probe!\r\n");
    /* 搭建字符设备框架 */
    /* 1、创建设备号 */
	if (ap3216cdev.major) {		/*  定义了设备号 */
		ap3216cdev.devid = MKDEV(ap3216cdev.major, 0);
		ret = register_chrdev_region(ap3216cdev.devid, AP3216C_CNT, AP3216C_NAME);
	} else {						/* 没有定义设备号 */
		ret = alloc_chrdev_region(&ap3216cdev.devid, 0, AP3216C_CNT, AP3216C_NAME);	/* 申请设备号 */
		ap3216cdev.major = MAJOR(ap3216cdev.devid);	/* 获取分配号的主设备号 */
		ap3216cdev.minor = MINOR(ap3216cdev.devid);	/* 获取分配号的次设备号 */
	}
    if(ret < 0){
        goto fail_devid;
    }
	printk("ap3216c major=%d,minor=%d\r\n",ap3216cdev.major, ap3216cdev.minor);	
	
	/* 2、初始化cdev */
	ap3216cdev.cdev.owner = THIS_MODULE;
	cdev_init(&ap3216cdev.cdev, &ap3216c_fops);
	
	/* 3、添加一个cdev */
	ret = cdev_add(&ap3216cdev.cdev, ap3216cdev.devid, AP3216C_CNT);
    if(ret < 0){
        goto fail_cdev;
    } 
	/* 4、创建类 */
	ap3216cdev.class = class_create(THIS_MODULE, AP3216C_NAME);
	if(IS_ERR(ap3216cdev.class)){
        ret = PTR_ERR(ap3216cdev.class);
        goto fail_classcrt;
    }

	/* 5、创建设备 */
	ap3216cdev.device = device_create(ap3216cdev.class, NULL, ap3216cdev.devid, NULL, AP3216C_NAME);
	if(IS_ERR(ap3216cdev.device)){
        ret = PTR_ERR(ap3216cdev.device);
        goto fail_devicecrt;
    }

    ap3216cdev.private_data = client;

    return 0;
fail_devicecrt:
    class_destroy(ap3216cdev.class);
fail_classcrt:
    cdev_del(&ap3216cdev.cdev);
fail_cdev:
    unregister_chrdev_region(ap3216cdev.devid, AP3216C_CNT);
fail_devid:
    return ret;

}

static int ap3216c_i2c_remove(struct i2c_client *i2c){
    
    cdev_del(&ap3216cdev.cdev);
    unregister_chrdev_region(ap3216cdev.devid, AP3216C_CNT);

    device_destroy(ap3216cdev.class, ap3216cdev.devid);
    class_destroy(ap3216cdev.class);
   
    return 0;
}


/* i2c_driver */
static struct i2c_driver ap3216c_driver = {
    .probe = ap3216c_i2c_probe,
    .remove = ap3216c_i2c_remove,
    .driver = {
        .name = "ap3216c",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ap3216c_of_match),
    },
    .id_table = ap3216c_id,
};


static int __init ap3216c_init(void){
    int ret = 0;
    ret = i2c_add_driver(&ap3216c_driver);
    return ret;
}

static void __exit ap3216c_exit(void){
    i2c_del_driver(&ap3216c_driver);
}

module_init(ap3216c_init);
module_exit(ap3216c_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
