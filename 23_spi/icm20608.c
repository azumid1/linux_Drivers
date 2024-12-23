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
#include<linux/regmap.h>

#define ICM20608_CNT     1
#define ICM20608_NAME    "icm20608"

struct icm20608_dev{
    int major;
    int minor;
    dev_t devid;
    struct class *class;
    struct device *device;
    struct cdev cdev;
    void *private_data;
    int cs_gpio;
    struct device_node *nd;

    signed int gyro_x_adc;		/* 陀螺仪X轴原始值 	 */
	signed int gyro_y_adc;		/* 陀螺仪Y轴原始值		*/
	signed int gyro_z_adc;		/* 陀螺仪Z轴原始值 		*/
	signed int accel_x_adc;		/* 加速度计X轴原始值 	*/
	signed int accel_y_adc;		/* 加速度计Y轴原始值	*/
	signed int accel_z_adc;		/* 加速度计Z轴原始值 	*/
	signed int temp_adc;		/* 温度原始值 			*/  

    /* 使用Regmap API */
    struct regmap *regmap;
    struct regmap_config regmap_config;


};

static struct icm20608_dev icm20608dev;

/* spi读寄存器 */
static int icm20608_read_regs(struct icm20608_dev *dev, u8 reg, void *buf, int len){
    u8 data = 0;
    /* 提取私有数据 */
    struct spi_device *spi = (struct spi_device *)dev->private_data;
    /* 片选拉低 */
    // gpio_set_value(dev->cs_gpio, 0);
    data = reg | 0x80;
    spi_write_then_read(spi, &data, 1, buf, len);
    // spi_write(spi, &data, 1);   /* 发送要读取的地址,这里的1表示发送的字节长度为1 */

    // spi_read(spi, buf, len);    /* 读取数据 */

    /* 片选拉高,意味着读寄存器结束 */
    // gpio_set_value(dev->cs_gpio, 1);
    return 0;
}

/* spi写寄存器 */
static int icm20608_write_regs(struct icm20608_dev *dev, u8 reg, u8 *buf, int len){
    u8 data = 0;
    u8 *txdata;
    /* 提取私有数据 */
    struct spi_device *spi = (struct spi_device *)dev->private_data;
    /* 片选拉低 */
    // gpio_set_value(dev->cs_gpio, 0);

    txdata = kzalloc(len + 1, GFP_KERNEL);
    txdata[0] = reg & ~0x80;    /* 要写的寄存器地址 */
    memcpy(&txdata[1], buf, len);   /* 要发送的数据拷贝到txdata里 */
    spi_write(spi, txdata, len + 1);
    // spi_write(spi, &data, 1);   /* 发送要写的地址,这里的1表示发送的字节长度为1，这个长度是固定的，
    //                                 因为spi的设备地址是7位的，然后在最前面追加了一个标志位，来确定读和写 */

    // spi_write(spi, buf, len);   /* 发送要写的数据 */    

    kfree(txdata);

    /* 片选拉高,意味着读寄存器结束 */
    // gpio_set_value(dev->cs_gpio, 1);
    return 0;
}

/* icm20608读取单个寄存器 */
static unsigned char icm20608_read_onereg(struct icm20608_dev *dev, u8 reg){
    unsigned int data = 0;
    u8 ret;
    //icm20608_read_regs(dev, reg, &data, 1);
    ret = regmap_read(dev->regmap, reg, &data);
    return data;

}

/* icm20608写单个寄存器 */
static void icm20608_write_onereg(struct icm20608_dev *dev, u8 reg, u8 value){
    //u8 buf = value;
    //icm20608_write_regs(dev, reg, &buf, 1);
    regmap_write(dev->regmap, reg, value);
    
}


/*
 * @description	: 读取ICM20608的数据，读取原始数据，包括三轴陀螺仪、
 * 				: 三轴加速度计和内部温度。
 * @param - dev	: ICM20608设备
 * @return 		: 无。
 */
void icm20608_readdata(struct icm20608_dev *dev)
{
	unsigned char data[14];
    u8 ret;
	//icm20608_read_regs(dev, ICM20_ACCEL_XOUT_H, data, 14);
    
    ret = regmap_bulk_read(dev->regmap, ICM20_ACCEL_XOUT_H, data, 14);

	dev->accel_x_adc = (signed short)((data[0] << 8) | data[1]); 
	dev->accel_y_adc = (signed short)((data[2] << 8) | data[3]); 
	dev->accel_z_adc = (signed short)((data[4] << 8) | data[5]); 
	dev->temp_adc    = (signed short)((data[6] << 8) | data[7]); 
	dev->gyro_x_adc  = (signed short)((data[8] << 8) | data[9]); 
	dev->gyro_y_adc  = (signed short)((data[10] << 8) | data[11]);
	dev->gyro_z_adc  = (signed short)((data[12] << 8) | data[13]);
}


/* icm20608寄存器初始化 */ 
void icm20608_reg_init(struct icm20608_dev *dev){
    u8 value = 0;
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x80);     /* 复位，复位后为0x40,睡眠模式 */
    mdelay(50);
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x01);     /* 关闭睡眠，自动选择时钟 */
    mdelay(50);
    printk("icm20608_reg_intit!\r\n");
    value = icm20608_read_onereg(dev, ICM20_WHO_AM_I);
    printk("ICM20608 ID = %#X\r\n", value);
    value = icm20608_read_onereg(dev, ICM20_PWR_MGMT_1);
    printk("ICM20_PWR_MGMT_1 ID = %#X\r\n", value);

    icm20608_write_onereg(&icm20608dev, ICM20_SMPLRT_DIV, 0x00); 	/* 输出速率是内部采样率					*/
	icm20608_write_onereg(&icm20608dev, ICM20_GYRO_CONFIG, 0x18); 	/* 陀螺仪±2000dps量程 				*/
	icm20608_write_onereg(&icm20608dev, ICM20_ACCEL_CONFIG, 0x18); 	/* 加速度计±16G量程 					*/
	icm20608_write_onereg(&icm20608dev, ICM20_CONFIG, 0x04); 		/* 陀螺仪低通滤波BW=20Hz 				*/
	icm20608_write_onereg(&icm20608dev, ICM20_ACCEL_CONFIG2, 0x04); /* 加速度计低通滤波BW=21.2Hz 			*/
	icm20608_write_onereg(&icm20608dev, ICM20_PWR_MGMT_2, 0x00); 	/* 打开加速度计和陀螺仪所有轴 				*/
	icm20608_write_onereg(&icm20608dev, ICM20_LP_MODE_CFG, 0x00); 	/* 关闭低功耗 						*/
	icm20608_write_onereg(&icm20608dev, ICM20_FIFO_EN, 0x00);		/* 关闭FIFO	 */
}

static int icm20608_open(struct inode *inode, struct file *filp){
    filp->private_data = &icm20608dev; /* 设置私有数据 */
    return 0;
}

static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
    signed int data[7];
	long err = 0;
	struct icm20608_dev *dev = (struct icm20608_dev *)filp->private_data;

	icm20608_readdata(dev);
	data[0] = dev->gyro_x_adc;
	data[1] = dev->gyro_y_adc;
	data[2] = dev->gyro_z_adc;
	data[3] = dev->accel_x_adc;
	data[4] = dev->accel_y_adc;
	data[5] = dev->accel_z_adc;
	data[6] = dev->temp_adc;
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

#if 0
/* spi读寄存器 */
static int icm20608_read_regs(struct icm20608_dev *dev, u8 reg, void *buf, int len){
    int ret = 0;
    unsigned char txdata[len];

    struct spi_message m;
    struct spi_transfer *t;
    /* 提取私有数据 */
    struct spi_device *spi = (struct spi_device *)dev->private_data;

    /* 片选拉低 */
    gpio_set_value(dev->cs_gpio, 0);

    /* 构建transfer */
    t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    /* 第一步：发送要读取的寄存器地址 */
    /* Read (1) or Write (0) operation */
    txdata[0] = reg | 0x80;
    t->tx_buf = txdata;
    t->len = 1;

    spi_message_init(&m);

    /* spi_message 初始化完成以后需要将 spi_transfer 添加到 spi_message 队列 */
    spi_message_add_tail(t, &m);

    ret = spi_sync(spi, &m);

    /* 第二步，读取数据 */
    txdata[0] = 0xff;   /* 这是无效的 */
    t->rx_buf = buf;
    t->len = len;

    spi_message_init(&m);
    spi_message_add_tail(t, &m);
    ret = spi_sync(spi, &m);

    kfree(t);

    /* 片选拉高,意味着读寄存器结束 */
    gpio_set_value(dev->cs_gpio, 1);

   
}

/* spi写寄存器 */
static int icm20608_write_regs(struct icm20608_dev *dev, u8 reg, u8 *buf, int len){
    int ret = 0;
    unsigned char txdata[len];

    struct spi_message m;
    struct spi_transfer *t;
    /* 提取私有数据 */
    struct spi_device *spi = (struct spi_device *)dev->private_data;

    /* 片选拉低 */
    gpio_set_value(dev->cs_gpio, 0);

    /* 构建transfer */
    t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    /* 第一步：发送要写的寄存器地址 */
    /* Read (1) or Write (0) operation */
    txdata[0] = reg & ~0x80;     /* 这里0x80取反，第一位是0，再做一个与操作，就满足了Write (0) */
    t->tx_buf = txdata;
    t->len = 1;

    spi_message_init(&m);

    /* spi_message 初始化完成以后需要将 spi_transfer 添加到 spi_message 队列 */
    spi_message_add_tail(t, &m);

    ret = spi_sync(spi, &m);

    /* 第二步，发送数据 */
    t->tx_buf = buf;
    t->len = len;

    spi_message_init(&m);
    spi_message_add_tail(t, &m);
    ret = spi_sync(spi, &m);

    kfree(t);

    /* 片选拉高,意味着读寄存器结束 */
    gpio_set_value(dev->cs_gpio, 1);
    return ret;
}

#endif



static int icm20608_probe(struct spi_device *spi){
    int ret = 0;
    printk("icm20608_probe\r\n");

    /* 初始化regmap_config设置 */
    icm20608dev.regmap_config.reg_bits = 8;    /* 寄存器地址长度为 8bit */
    icm20608dev.regmap_config.val_bits = 8;    /* 值长度 8bit */
    icm20608dev.regmap_config.read_flag_mask = 0x80;   /* 读掩码 */

    /* 初始化spi接口的regmap */
    icm20608dev.regmap = regmap_init_spi(spi, &icm20608dev.regmap_config);
    if(IS_ERR(icm20608dev.regmap)){
        return PTR_ERR(icm20608dev.regmap);
    }

    /* 搭建字符设备框架 */
    /* 1、创建设备号 */
	if (icm20608dev.major) {		/*  定义了设备号 */
		icm20608dev.devid = MKDEV(icm20608dev.major, 0);
		ret = register_chrdev_region(icm20608dev.devid, ICM20608_CNT, ICM20608_NAME);
	} else {						/* 没有定义设备号 */
		ret = alloc_chrdev_region(&icm20608dev.devid, 0, ICM20608_CNT, ICM20608_NAME);	/* 申请设备号 */
		icm20608dev.major = MAJOR(icm20608dev.devid);	/* 获取分配号的主设备号 */
		icm20608dev.minor = MINOR(icm20608dev.devid);	/* 获取分配号的次设备号 */
	}
    if(ret < 0){
        goto del_regmap;
    }
	printk("icm20608 major=%d,minor=%d\r\n",icm20608dev.major, icm20608dev.minor);	
	
	/* 2、初始化cdev */
	icm20608dev.cdev.owner = THIS_MODULE;
	cdev_init(&icm20608dev.cdev, &icm20608_fops);
	
	/* 3、添加一个cdev */
	ret = cdev_add(&icm20608dev.cdev, icm20608dev.devid, ICM20608_CNT);
    if(ret < 0){
        goto fail_cdev;
    } 
	/* 4、创建类 */
	icm20608dev.class = class_create(THIS_MODULE, ICM20608_NAME);
	if(IS_ERR(icm20608dev.class)){
        ret = PTR_ERR(icm20608dev.class);
        goto fail_classcrt;
    }

	/* 5、创建设备 */
	icm20608dev.device = device_create(icm20608dev.class, NULL, icm20608dev.devid, NULL, ICM20608_NAME);
	if(IS_ERR(icm20608dev.device)){
        ret = PTR_ERR(icm20608dev.device);
        goto fail_devicecrt;
    }

#if 0
    /* 获取片选引脚 */
    /* spi->dev.of_node的父节点存储着引脚信息 */
    icm20608dev.nd = of_get_parent(spi->dev.of_node);
    icm20608dev.cs_gpio = of_get_named_gpio(icm20608dev.nd, "cs-gpio", 0);
    if(icm20608dev.cs_gpio < 0){
        printk("get cs-gpio failed!\r\n");
        goto fail_gpio;
    }

    /* 申请使用这个引脚 */
    ret = gpio_request(icm20608dev.cs_gpio, "cs_gpio");
    if(ret < 0) printk("gpio requeset failed!\r\n");

    ret = gpio_direction_output(icm20608dev.cs_gpio, 1);    /* 默认输出高电平，因为现在还没有开始通信，如果输出低电平，就表示开始通信了 */
#endif

    /* 初始化spi_device 的模式 */
    spi->mode = SPI_MODE_0;
    spi_setup(spi);

    /* 设置私有数据 */
    icm20608dev.private_data = spi;
    printk("111\r\n");
    /* icm20608初始化 */
    icm20608_reg_init(&icm20608dev);
    printk("222\r\n");

    return 0;


fail_devicecrt:
    class_destroy(icm20608dev.class);
fail_classcrt:
    cdev_del(&icm20608dev.cdev);
fail_cdev:
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);
del_regmap:
    regmap_exit(icm20608dev.regmap);
    return ret;

}

static int icm20608_remove(struct spi_device *spi){
    cdev_del(&icm20608dev.cdev);
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);

    // gpio_free(icm20608dev.cs_gpio);

    device_destroy(icm20608dev.class, icm20608dev.devid);
    class_destroy(icm20608dev.class);

    regmap_exit(icm20608dev.regmap);

    return 0;
}


/* 设备树匹配表 */
static const struct of_device_id icm20608_of_match[] = {
    {.compatible = "alientek,icm20608"},
    {}
};

/* 传统的匹配表 */
static struct spi_device_id icm20608_id[] = {
    {"alientek,icm20608",0},
    {}
};


/* spi_driver */
struct spi_driver icm20608drv = {
    .probe = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "icm20608",
        .of_match_table = icm20608_of_match,
    },
    .id_table = icm20608_id,

};

static int __init icm20608_init(void){
    int ret = 0;
    ret = spi_register_driver(&icm20608drv);
    return ret;
}

static void __exit icm20608_exit(void){
   spi_unregister_driver(&icm20608drv);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
