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
#include<linux/spi/spi.h>
#include<linux/delay.h>
#include "icm20608reg.h"

#define ICM20608_CNT    1
#define ICM20608_NAME   "icm20608"

struct icm20608_dev{
    int major;
    int minor;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    void *private_data;
    int cs_gpio;    /* 用来存储引脚 */

    signed int gyro_x_adc;		/* 陀螺仪X轴原始值 	 */
	signed int gyro_y_adc;		/* 陀螺仪Y轴原始值		*/
	signed int gyro_z_adc;		/* 陀螺仪Z轴原始值 		*/
	signed int accel_x_adc;		/* 加速度计X轴原始值 	*/
	signed int accel_y_adc;		/* 加速度计Y轴原始值	*/
	signed int accel_z_adc;		/* 加速度计Z轴原始值 	*/
	signed int temp_adc;		/* 温度原始值 			*/    

};

static struct icm20608_dev icm20608dev;

/* spi读寄存器 */
static int icm20608_read_regs(struct icm20608_dev *dev, u8 reg, void *buf, int len){
    u8 data = 0;
    struct spi_device *spi = (struct spi_device*)dev->private_data;
    data = reg | 0x80;
    spi_write_then_read(spi, &data, 1, buf, len);

    return 0;
}

/* spi写寄存器 */
static int icm20608_write_regs(struct icm20608_dev *dev, u8 reg, u8 *buf, int len){
    u8 data = 0;
    u8 *txdata;
    struct spi_device *spi = (struct spi_device*)dev->private_data;
    txdata = kzalloc(len + 1, GFP_KERNEL);
    txdata[0] = reg & ~0x80;
    memcpy(&txdata[1], buf, len);
    spi_write(spi, txdata, len + 1);

    kfree(txdata);  /* 申请的内存不用了一定要释放 */

    return 0;
}



/* icm20608读取单个寄存器 */
static unsigned char icm20608_read_onereg(struct icm20608_dev *dev, u8 reg){
    u8 data = 0;
    icm20608_read_regs(dev, reg, &data, 1); 
    return data;
}

/* icm20608写单个寄存器 */
static void icm20608_write_onereg(struct icm20608_dev *dev, u8 reg, u8 value){
    u8 buf = value;
    icm20608_write_regs(dev, reg, &buf, 1);
}

void icm20608_readdata(struct icm20608_dev *dev){
    unsigned char data[14];
    icm20608_read_regs(dev, ICM20_ACCEL_XOUT_H, data, 14);

    dev->accel_x_adc = (signed short)((data[0] << 8) | data[1]);
    dev->accel_y_adc = (signed short)((data[2] << 8) | data[3]);
    dev->accel_z_adc = (signed short)((data[4] << 8) | data[5]);
    dev->temp_adc = (signed short)((data[6] << 8) | data[7]);
    dev->gyro_x_adc = (signed short)((data[8] << 8) | data[9]);
    dev->gyro_y_adc = (signed short)((data[10] << 8) | data[11]);
    dev->gyro_z_adc = (signed short)((data[12] << 8) | data[13]);
}

void icm20608_reg_init(struct icm20608_dev *dev){
    u8 value = 0;
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x80); /* 复位，复位后为0x40,睡眠模式 */
    mdelay(50);
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x01); /* 关闭睡眠，自动选择时钟 */
    mdelay(50);
    printk("icm20608_reg_init!\r\n");
    value = icm20608_read_onereg(dev, ICM20_WHO_AM_I);
    printk("ICM20_WHO_AM_I = %#X\r\n", value);

    icm20608_write_onereg(dev, ICM20_SMPLRT_DIV, 0x00);         /* 输出速率是内部采样率	         */
    icm20608_write_onereg(dev, ICM20_CONFIG, 0x04);             /* 陀螺仪低通滤波BW=20Hz        */
    icm20608_write_onereg(dev, ICM20_GYRO_CONFIG, 0x18);        /* 陀螺仪±2000dps量程           */
    icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG, 0x18);       /* 加速度计±16G量程             */
    icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG2, 0x04);      /* 加速度计低通滤波BW=21.2Hz    */
    icm20608_write_onereg(dev, ICM20_LP_MODE_CFG, 0x00);        /* 关闭低功耗                   */
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_2, 0x00);         /* 打开加速度计和陀螺仪所有轴    */
    icm20608_write_onereg(&icm20608dev, ICM20_FIFO_EN, 0x00);	/* 关闭FIFO	 */	

}

static int icm20608_open(struct inode *inode, struct file *filp){
    filp->private_data = &icm20608dev;
    return 0;
}

static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
    signed int data[7];
    long err = 0;
    struct icm20608_dev *dev = (struct icm20608_dev*)filp->private_data;

    icm20608_readdata(dev);
    data[0] = dev->accel_x_adc;
    data[1] = dev->accel_y_adc;
    data[2] = dev->accel_z_adc;
    data[3] = dev->temp_adc;
    data[4] = dev->gyro_x_adc;
    data[5] = dev->gyro_y_adc;
    data[6] = dev->gyro_z_adc;\
    err = copy_to_user(buf, data, sizeof(data));
    return 0;
}

static int icm20608_release(struct inode *inode, struct file *filp){
    return 0;
}

static struct file_operations icm20608_fops = {
    .owner = THIS_MODULE,
    .open = icm20608_open,
    .read = icm20608_read,
    .release = icm20608_release,
};

static int icm20608_probe(struct spi_device *spi){
    int ret = 0;
    printk("icm20608_probe!\r\n");

    /* 搭建字符设备驱动框架，在/dev/ */
    /* 注册字符设备 */
    if(icm20608dev.major){
        icm20608dev.devid = MKDEV(icm20608dev.major,0);
        ret = register_chrdev_region(icm20608dev.devid, ICM20608_CNT, ICM20608_NAME);
    } else{
        ret = alloc_chrdev_region(&icm20608dev.devid, 0, ICM20608_CNT, ICM20608_NAME);
        icm20608dev.major = MAJOR(icm20608dev.devid);
        icm20608dev.minor = MINOR(icm20608dev.devid);
    }
    if(ret < 0) goto fail_devid;
    printk("icm20608 major=%d,minor=%d\r\n",icm20608dev.major, icm20608dev.minor);	

    /* 初始化cdev */
    icm20608dev.cdev.owner = THIS_MODULE;
    cdev_init(&icm20608dev.cdev, &icm20608_fops);

    /* 添加一个cdev */
    ret = cdev_add(&icm20608dev.cdev, icm20608dev.devid, ICM20608_CNT);
    if(ret < 0) goto fail_cdev;

    /* 创建类 */
    icm20608dev.class = class_create(THIS_MODULE, ICM20608_NAME);
    if(IS_ERR(icm20608dev.class)){
        ret = PTR_ERR(icm20608dev.class);
        goto fail_class;
    }

    /* 创建设备 */
    icm20608dev.device = device_create(icm20608dev.class, NULL, icm20608dev.devid, NULL, ICM20608_NAME);
    if(IS_ERR(icm20608dev.device)){
        ret = PTR_ERR(icm20608dev.device);
        goto fail_device;
    }

    /* 初始化spi_device 的模式 */
    spi->mode = SPI_MODE_0;
    spi_setup(spi);

    /* 设置私有数据 */
    icm20608dev.private_data = spi;

    /* icm20608初始化 */
    icm20608_reg_init(&icm20608dev);
    return 0;


fail_device:
    class_destroy(icm20608dev.class);
fail_class:
    cdev_del(&icm20608dev.cdev);
fail_cdev:
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);
fail_devid:
    return ret;
}

static int icm20608_remove(struct spi_device *spi){
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);
    cdev_del(&icm20608dev.cdev);

    device_destroy(icm20608dev.class, icm20608dev.devid);
    class_destroy(icm20608dev.class);
    return 0;
}

/* 传统匹配 */
struct spi_device_id icm20608_id[] = {
    {"alientek,icm20608", 0},
    {},
};

/* 设备树匹配 */
static const struct of_device_id icm20608_of_match[] = {
    {.compatible = "alientek,icm20608"},
    {},
};

struct spi_driver icm20608_driver = {
    .probe = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .name = "icm20608",
        .owner = THIS_MODULE,
        .of_match_table = icm20608_of_match,
    },
    .id_table = icm20608_id,
};


/* 驱动入口函数 */
static int __init icm20608_init(void){
    int ret = 0;
    ret = spi_register_driver(&icm20608_driver);
    return ret;
}

/* 驱动出口函数 */
static void __exit icm20608_exit(void){
    spi_unregister_driver(&icm20608_driver);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");