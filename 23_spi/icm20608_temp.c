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

#define ICM20608_CNT     1
#define ICM20608_NAME    "icm20608"

struct icm20608_dev{
    int major;
    int minor;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
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

struct icm20608_dev icm20608dev;

/* spi读寄存器 */
static int icm20608_read_regs(struct icm20608dev *dev, u8 reg, void *buf, int len){
    u8 data = 0;
    struct spi_device *spi = (struct spi_device*)dev->private_data;

    data = reg | 0x80;
    spi_write_then_read(spi, &data, 1, buf, len);
    return 0;
}

/* spi写寄存器 */
static int icm20608_write_regs(struct icm20608dev *dev, u8 reg, u8 *buf, int len){
    u8 data = 0;
    u8 *txdata;

    struct spi_device *spi = (struct spi_device*)dev->private_data;

    txdata = kzalloc(len+1, GFP_KERNEL);
    txdata[0] = reg & ~0x80;    /* 要写的寄存器的地址 */
    memcpy(&txdata[1], buf, len);
    spi_write(spi, txdata, len + 1);

    kfree(txdata);
    return 0;

}

static unsigned char icm20608_read_onereg(struct icm20608dev *dev, u8 reg){
    unsigned int data = 0;
    regmap_read(dev->regmap, reg, &data);
    return data;
}

static void icm20608_write_onereg(struct icm20608dev *dev, u8 reg, u8 value){
    regmap_write(dev->regmap, reg, value);
}

/* icm20608寄存器初始化 */
void icm20608_reg_init(struct icm20608dev *dev){
    u8 value = 0;
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x80); /* 复位 */
    mdelay(50);
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x01); /* 自动选择时钟，开启陀螺仪的全部性能 */
    mdelay(50);

    value = icm20608_read_onereg(dev, ICM20_PWR_WHO_AM_I);
    printk("ICM20608 ID = %#x\r\n", value);
    value = icm20608_read_onereg(dev, ICM20_PWR_MGMT_1);
    printk("ICM20_PWR_MGMT_1 ID = %#X\r\n", value);

    icm20608_write_onereg(dev, ICM20_SIMPLRT_DIV, 0x00);    /* 输出速率是内部采样率					*/
    icm20608_write_onereg(dev, ICM20_CONFIG,0x04);          /* 陀螺仪低通滤波BW=20Hz 				*/
    icm20608_write_onereg(dev, ICM20_GYRO_CONFIG, 0x18);    /* 陀螺仪±2000dps量程 				*/
    icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG, 0x18);   /* 加速度计±16G量程 					*/
    icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG2, 0x04);  /* 加速度计低通滤波BW=21.2Hz 			*/
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_2, 0x00);     /* 打开加速度计和陀螺仪所有轴 				*/
    icm20608_write_onereg(dev, ICM20_LP_MODE_CFG, 0x00);    /* 关闭低功耗 						*/
    icm20608_write_onereg(&icm20608dev, ICM20_FIFO_EN, 0x00);		/* 关闭FIFO	 */
}

void icm20608_readdata(struct icm20608dev *dev){
    unsigned char data[14];
    u8 ret;

    ret = regmap_bulk_read(dev->regmap, ICM20_ACCEL_XOUT_H, data, 14);

    dev->accel_x_adc = (signed short)((data[0] << 8) | data[1]); 
	dev->accel_y_adc = (signed short)((data[2] << 8) | data[3]); 
	dev->accel_z_adc = (signed short)((data[4] << 8) | data[5]); 
	dev->temp_adc    = (signed short)((data[6] << 8) | data[7]); 
	dev->gyro_x_adc  = (signed short)((data[8] << 8) | data[9]); 
	dev->gyro_y_adc  = (signed short)((data[10] << 8) | data[11]);
	dev->gyro_z_adc  = (signed short)((data[12] << 8) | data[13]);
}

static int icm20608_open(struct inode *inode, struct file *filp){
    filp->private_data = &icm20608dev;
}

static ssize_t icm20608_read(struct file *filp, char __user *buf, size_t cnt, lofft *offt){
    int err = 0;
    icm20608_readdata(dev);
	data[0] = dev->gyro_x_adc;
	data[1] = dev->gyro_y_adc;
	data[2] = dev->gyro_z_adc;
	data[3] = dev->accel_x_adc;
	data[4] = dev->accel_y_adc;
	data[5] = dev->accel_z_adc;
	data[6] = dev->temp_adc;
	err = copy_to_user(buf, data, sizeof(data));
    return err;
}

static int icm20608_release(struct inode *inode, struct file *filp){
    return 0;
}

static struct file_operatons icm20608_fops = {
    .owner = THIS_MODULE,
    .open = icm20608_open,
    .read = icm20608_read,
    .release = icm20608_release,
};

static int icm20608_probe(struct spi_device *spi){

    icm20608dev.regmap_config.reg_bits = 8;
    icm20608dev.regmap_config.val_bits = 8;
    icm20608dev.regmap_config.read_flag_mask = 0x80;    /* 读掩码 */

    /* 初始化spi接口的regmap */
    icm20608dev.regmap = regmap_init_spi(spi, &icm20608dev.regmap_config);
    if(IS_ERR(icm20608dev.regmap)){
        return PTR_ERR(icm20608dev.regmap);
    }

    if(icm20608dev.major){
        icm20608dev.devid = MKDEV(icm20608dev.major, 0);
        register_chrdev_region(icm20608dev.devid, ICM20608_CNT, ICM20608_NAME);
    } else{
        alloc_chrdev_region(&icm20608.devid, 0, ICM20608_CNT, ICM20608_NAME);
        icm20608dev.major = MAJOR(icm20608dev.devid);
        icm20608dev.minor = MINOR(icm20608dev.devid);
    }

    icm20608dev.cdev.owner = THIS_MODULE;
    cdev_init(&icm20608dev.cdev, &icm20608_fops);
    cdev_add(&icm20608dev.cdev, icm20608dev.devid, ICM20608_CNT);

    icm20608dev.class = class_create(THIS_MODULE, ICM2060_NAME);
    icm20608dev.device = device_create(icm20608dev.class, NULL, icm20608dev.devid, NULL, ICM20608_NAME);

    spi->mode = SPI_MODE_0;
    spi_setup(spi);

    icm20608dev->private_data = spi;
    icm20608_reg_init(&icm20608dev);

    return 0;
}

static int icm20608_remove(struct spi_device *spi){
    cdev_del(&icm20608dev.cdev);
    unregister_chrdev_region(icm20608dev.devid, ICM20608_CNT);

    device_destroy(icm20608dev.class, icm20608dev.devid);
    class_destroy(icm20608dev.class);
    
    regmap_exit(icm20608dev.regmap);
}

sataic const struct of_device_id icm20608_of_match[] = {
    {.compatible="alientek,icm20608"},
    {}
};

static struct spi_device_id icm20608_id[] = {
    {"alientek,icm20608", 0},
    {}
};

struct spi_driver icm20608drv = {
    .probe = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .name = "icm20608",
        .owner = THIS_MODULE,
        .of_math_table = icm20608_of_match,
    },
    .id_table = icm20608_id,
};

static int __init icm20608_init(void){
    spi_register_driver(&icm20608drv);
}

staic void __exit icm20608_exit(void){
    spi_unregister_driver(&icm20608drv);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");