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

#define AP3216C_NAME "ap3216c"
#define AP3216C_CNT  1

struct ap3216c_dev{
    dev_t devid;
    int major;
    int minor;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    void *private_data;
    unsigned short ir,als,ps;
};

struct ap3216c_dev ap3216c;

/* 读取ap3216c的N个寄存器 */
static int ap3216c_read_regs(struct ap3216c_dev *dev, u8 reg, void *val, int len){
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client*)dev->private_data;

    /* msg[0]发送要读取寄存器的地址 */
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = 1;



    /* msg[1]用来读取数据 */
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = val;
    msg[1].len = len;
    return i2c_transfer(client->adapter, msg, 2);

}

/* 写入ap3216c的N个寄存器 */
static int ap3216c_write_regs(struct ap3216c_dev *dev, u8 *reg, u8 *buf, u8 len){
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client*)dev->private_data;

    b[0] = reg;
    memcpy(&buf[1], buf, len);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.buf = b;
    msg.len = len + 1;

    return i2c_transfer(client->adapter, &msg, 1);
}

/* 读ap3216c一个寄存器 */
static unsigned char ap3216c_read_reg(struct ap3216c_dev *dev, u8 reg){
    u8 data = 0;
    ap3216c_read_regs(dev, reg, &data, 1);
    return data;
}

/* 向ap3216c一个寄存器写数据 */
static void ap3216c_write_reg(struct ap3216c_dev *dev, u8 reg, u8 data){
    u8 buf = 0;
    buf = data;
    ap3216c_write_regs(dev, &reg, &buf, 1);
}

void ap3216c_readdata(struct ap3216c_dev *dev){
    unsigned char buf[6];
    unsigned char i;
    
    for(i=0; i<6; i++){
        buf[i] = ap3216c_read_reg(dev, AP3216C_IRDATALOW+i);
    }
    if(buf[0] & 0x80){
        dev->ir = 0;
        dev->ps = 0;
    } else{
        dev->ir = (((unsigned short)buf[1] << 2) | (buf[0] & 0x03));
        dev->ps = (((unsigned short)buf[5] & 0x3F) << 4) | (buf[4] & 0x0F);
    }

    dev->als = ((unsigned short)buf[3] << 8) | buf[2];
}

static int ap3216c_open(struct inode *inode, struct file *filp){
    unsigned char value = 0;
    filp->private_data = &ap3216c;
    printk("ap3216c_open!\r\n");

    /* 初始化ap3216c */
    ap3216c_write_reg(&ap3216c, AP3216C_SYSTEMCONG, 0x04);
    mdelay(50);
    ap3216c_write_reg(&ap3216c, AP3216C_SYSTEMCONG, 0x03);
    value = ap3216c_read_reg(&ap3216c, AP3216C_SYSTEMCONG);
    pritnk("AP3216C_SYSTEMCONG = %#x\r\n", value);
    return 0;
}

static ssize_t ap3216c_read(struct file filp, char __user *buf, size_t cnt, lofft *offt){
    long err = 0;
    short data[3];
    struct ap3216c_dev *dev = (struct ap3216c_dev*)filp->private_data;
    printk("ap3216c_read!\r\n");

    /* 读取ap3216c中的原始数据 */
    data[0] = dev->ir;
    data[1] = dev->als;
    data[2] = dev->ps;

    err = copy_to_user(buf, data, sizeof(data));
    return 0;

}

static int ap3216c_release(struct inode *inode, struct file *filp){
    printk("ap3216c_release!\r\n");
    return 0;
}

static struct file_operations ap3216c_fops = {
    .owner = THIS_MODULE,
    .open = ap3216c_open,
    .read = ap3216c_read,
    .release = ap3216c_release,
};

static int ap3216c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id){
    int ret;
    printk("ap3216c_i2c_probe!\r\n");
    if(ap3216c.major){
        ap3216c.devid = MKDEV(ap3216c.major, 0);
        ret = register_chrdev_region(ap3216c.devid, AP3216C_CNT, AP3216C_NAME);
    } else{
        ret = alloc_chrdev_region(ap3216c.devid, 0, AP3216C_CNT, AP3216C_NAME);
        ap3216c.major = MAJOR(ap3216c.devid);
        ap3216c.minor = MINOR(ap3216c.devid);
    }

    printk("ap3216c major=%d,minor=%d\r\n",ap3216cdev.major, ap3216cdev.minor);	

    ap3216c.cdev.owner = THIS_MODULE;
    cdev_init(&ap3216c.cdev, &ap3216c_fops);
    ret = cdev_add(&ap3216c.cdev, ap3216c.devid, AP3216C_CNT);

    ap3216c.class = class_create(THIS_MODULE, AP3216C_NAME);
    ap3216c.device = device_create(ap3216c.class, NULL, ap3216c.devid, NULL, AP3216C_NAME);

    ap3216c.private_data = client;

    return 0;
}

static int ap3216c_i2c_remove(struct i2c_client *client){
    cdev_del(&ap3216c.cdev);
    unregister_chrdev_region(ap3216c.devid, AP3216C_CNT);
    device_destroy(ap3216c.class, ap3216c.devid);
    class_destroy(ap3216c.class);
    return 0;
}

static struct i2c_device_id ap3216c_id[] = {
    {"alientek,ap3216c", 0},
    {}
};

static struct of_device_id ap3216c_of_match[] = {
    {.compatible="alientek,ap3216c"},
    {}
};

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