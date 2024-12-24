#include <linux/spi/spi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include "icm20608reg.h"
#include <linux/gpio.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/unaligned/be_byteshift.h>

#define ICM20608_NAME   "icm20608"
#define ICM20608_TEMP_OFFSET    0
#define ICM20608_TEMP_SCALE     326800000

#define ICM20608_CHAN(_type, _channel2, _index)                     \
    {                                                               \
        .type = _type,                                              \
        .modified = 1,                                              \
        .channel2 = _channel2,                                      \
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),       \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	            \
				      BIT(IIO_CHAN_INFO_CALIBBIAS),                 \
		.scan_index = _index,                                       \
		.scan_type = {                                              \
				.sign = 's',                                        \
				.realbits = 16,                                     \
				.storagebits = 16,                                  \
				.shift = 0,                                         \
				.endianness = IIO_BE,                               \
			    },                                                  \
    }

/* 设备结构体 */
struct icm20608_dev {
    struct spi_device *spi;
    struct regmap *regmap;
    struct regmap_config regmap_config;
    struct mutex lock;
};


/* 
 * icm20608加速度计分辨率，对应2、4、8、16 计算方法：
 * 以正负2g量程为例，4/2^16=0.000061035，扩大1000000000倍，就是61035
 */
static const int accel_scale_icm20608[] = {61035, 122070, 244140, 488281};

/*
 * 陀螺仪分辨率吧，对应250,500,1000,2000, 计算方法：
 * 以正负250度量程为例，500/2^16=0.06719，扩大1000000倍，就是7269
 */
static const int gyro_scale_icm20608[] = {7629, 15258, 30517, 61035};

/* 
 * ICM20608的扫描元素，3轴加速度计、
 * 3轴陀螺仪、1路温度传感器，1路时间戳 
 */
enum imv_icm20608_scan {
    INV_ICM20608_SCAN_ACCL_X,
    INV_ICM20608_SCAN_ACCL_Y,
    INV_ICM20608_SCAN_ACCL_Z,
    INV_ICM20608_SCAN_TEMP,
    INV_ICM20608_SCAN_GYRO_X,
    INV_ICM20608_SCAN_GYRO_Y,
    INV_ICM20608_SCAN_GYRO_Z,
    INV_ICM20608_SCAN_TIMESTAMP,
};

/* icm20608通道，1路温度通道，3路加速度计，3路陀螺仪 */
static const struct iio_chan_spec icm20608_channels[] = {
    /* 温度通道 */
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
                            | BIT(IIO_CHAN_INFO_OFFSET)
                            | BIT(IIO_CHAN_INFO_SCALE),
        .scan_index = INV_ICM20608_SCAN_TEMP,
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_BE,
        },
    },

    ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_ICM20608_SCAN_GYRO_X),  /* 陀螺仪x轴 */
    ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_ICM20608_SCAN_GYRO_Y),   /* 陀螺仪y轴 */
    ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_ICM20608_SCAN_GYRO_Z),   /* 陀螺仪z轴 */

    /* 加速度x,y,z轴 */
    ICM20608_CHAN(IIO_ACCEL, IIO_MOD_X, INV_ICM20608_SCAN_ACCL_X),
    ICM20608_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_ICM20608_SCAN_ACCL_Y),
    ICM20608_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_ICM20608_SCAN_ACCL_Z),

};


/*
 * @description	: 向icm20608指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  icm20608设备
 * @param - reg:  要写的寄存器
 * @param - data: 要写入的值
 * @return   :    无
 */	

static void icm20608_write_onereg(struct icm20608_dev *dev, u8 reg, u8 value){
    regmap_write(dev->regmap, reg, value);
}

/*
 * @description	: 读取icm20608指定寄存器值，读取一个寄存器
 * @param - dev:  icm20608设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
static unsigned char icm20608_read_onereg(struct icm20608_dev *dev, u8 reg){
    unsigned int data;
    u8 ret = 0;
    ret = regmap_read(dev->regmap, reg, &data);
    return (u8)data;
}

/*
 * @description  	: ICM20608内部寄存器初始化函数 
 * @param - spi 	: 要操作的设备
 * @return 			: 无
 */
void icm20608_reginit(struct icm20608_dev *dev){
    u8 value = 0;
    /* 先复位 */
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x80);
    mdelay(50);
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x01);
    mdelay(50);

    value = icm20608_read_onereg(dev, ICM20_PWR_MGMT_1);
    printk("ICM20608 ID = %#x\r\n", value);

	icm20608_write_onereg(dev, ICM20_SMPLRT_DIV, 0x00); 	/* 输出速率是内部采样率		*/
	icm20608_write_onereg(dev, ICM20_GYRO_CONFIG, 0x18); 	/* 陀螺仪±2000dps量程 		*/
	icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG, 0x18); 	/* 加速度计±16G量程 		*/
	icm20608_write_onereg(dev, ICM20_CONFIG, 0x04); 		/* 陀螺仪低通滤波BW=20Hz 	*/
	icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG2, 0x04); /* 加速度计低通滤波BW=21.2Hz 	*/
	icm20608_write_onereg(dev, ICM20_PWR_MGMT_2, 0x00); 	/* 打开加速度计和陀螺仪所有轴 	*/
	icm20608_write_onereg(dev, ICM20_LP_MODE_CFG, 0x00); 	/* 关闭低功耗 				*/
	icm20608_write_onereg(dev, ICM20_INT_ENABLE, 0x01);		/* 使能FIFO溢出以及数据就绪中断	*/
}

/*
  * @description  	: 读取ICM20608传感器数据，可以用于陀螺仪、加速度计、温度的读取
  * @param - dev	: icm20608设备 
  * @param - reg  	: 要读取的通道寄存器首地址。
  * @param - anix  	: 需要读取的通道，比如X，Y，Z。
  * @param - val  	: 保存读取到的值。
  * @return			: 0，成功；其他值，错误
  */
 static int icm20608_sensor_show(struct icm20608_dev *dev, int reg,
				   int axis, int *val)
{
    int ind, result;
    __be16 d;

    /**
     * 这里对下面的代码做一下解释
     * 首先，axis代表的channel2，就是要读的是X Y Z 三轴中的一个
     * (axis - IIO_MOD_X) * 2 这是在计算要读的寄存器的首地址，为什么要*2呢？
     * 是因为icm20608的一个轴的数据是存放在两个寄存器里的，所以要想读取下一个轴的信息，
     * 就要向后跳两个寄存器。
     * 然后再使用regmap_bulk_read这个方法读取两个字节的内容，就得到对应的轴的数据了！！！
    */
    ind = (axis - IIO_MOD_X) * 2;
    result = regmap_bulk_read(dev->regmap, reg+ind, (u8 *)&d, 2);
    if(result){
        return -EINVAL;
    }
    *val = (short)be16_to_cpup(&d);
    return IIO_VAL_INT;
}

/*
  * @description  		: 读取ICM20608陀螺仪、加速度计、温度通道值
  * @param - indio_dev	: iio设备 
  * @param - chan  		: 通道。
  * @param - val  		: 保存读取到的通道值。
  * @return				: 0，成功；其他值，错误
  */
 static int icm20608_read_channel_data(struct iio_dev *indio_dev, struct iio_chan_spec const * chan,
                                        int *val)
{
    struct icm20608_dev *dev = iio_priv(indio_dev);
    int ret = 0;
    switch(chan->type){
        case IIO_ANGL_VEL:  /* 读取陀螺仪数据 */
            ret = icm20608_sensor_show(dev, ICM20_GYRO_XOUT_H, chan->channel2, val);
            break;
        case IIO_ACCEL:     /* 读取加速度计数据 */
            ret = icm20608_sensor_show(dev, ICM20_ACCEL_XOUT_H, chan->channel2, val);
            break;
        
        case IIO_TEMP:      /* 读取温度 */
            ret = icm20608_sensor_show(dev, ICM20_TEMP_OUT_H, IIO_MOD_X, val);
            break;
        default:
            ret = -EINVAL;
            break;
    } 
    return ret;  

}

static int icm20608_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
    struct icm20608_dev *dev = iio_priv(indio_dev);
    int ret = 0;
    unsigned char regdata = 0;
    
    /* 区分读取的文件类型，是raw，scale，还是offset等 */
    switch(mask){
        case IIO_CHAN_INFO_RAW:
            mutex_lock(&dev->lock);
            ret = icm20608_read_channel_data(indio_dev, chan, val);
            mutex_unlock(&dev->lock);
            return ret;
        case IIO_CHAN_INFO_SCALE:
            switch(chan->type){
                case IIO_ANGL_VEL:
                    mutex_lock(&dev->lock);
                    regdata = (icm20608_read_onereg(dev, ICM20_GYRO_CONFIG) & 0x18) >> 3;
                    *val = 0;
                    *val2 = gyro_scale_icm20608[regdata];
                    mutex_unlock(&dev->lock);
                    return IIO_VAL_INT_PLUS_MICRO;   /* 值为val+val2/1000000 */
                case IIO_ACCEL:
                    mutex_lock(&dev->lock);
                    regdata = (icm20608_read_onereg(dev, ICM20_ACCEL_CONFIG) & 0x18) >>3;
                    *val = 0;
                    *val2 = accel_scale_icm20608[regdata];
                    mutex_unlock(&dev->lock);
                    return IIO_VAL_INT_PLUS_NANO;  /* 值为val+val2/1000000000 */ 
                case IIO_TEMP:
                    /* 温度的量程是固定的 */
                    *val = ICM20608_TEMP_SCALE / 1000000;
                    *val2 = ICM20608_TEMP_SCALE % 1000000;
                    return IIO_VAL_INT_PLUS_MICRO;  /* 值为val+val2/1000000 */
                default:
                    return -EINVAL;
            }
            return ret;
        case IIO_CHAN_INFO_OFFSET:  /* ICM20608温度传感器offset值 */
            switch(chan->type){
                case IIO_TEMP:
                    *val = ICM20608_TEMP_OFFSET;
                    return IIO_VAL_INT;
                default:
                    return -EINVAL;
            }
            return ret;
        case IIO_CHAN_INFO_CALIBBIAS:   /* ICM20608加速度计和陀螺仪校准值 */
            switch(chan->type){
                case IIO_ANGL_VEL:  /* 陀螺仪的校准值 */
                    mutex_lock(&dev->lock);
                    ret = icm20608_sensor_show(dev, ICM20_XG_OFFS_USRH, chan->channel2, val);
                    mutex_unlock(&dev->lock);
                    return ret;
                case IIO_ACCEL: /* 加速度计的校准值 */
                    mutex_lock(&dev->lock);
                    ret = icm20608_sensor_show(dev, ICM20_XA_OFFSET_H, chan->channel2, val);
                    mutex_unlock(&dev->lock);
                    return ret;
                default:
                    return -EINVAL;
            }
        default:
            return -EINVAL;
    }
}

static int icm20608_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
    return 0;
}

static int icm20608_write_raw_get_fmt(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, long mask)
{
    return 0;
}
/**
 * iio_info结构体
*/
static struct iio_info icm20608_info = {
    .driver_module      = THIS_MODULE,
    .read_raw           = icm20608_read_raw,
    .write_raw          = icm20608_write_raw,
    .write_raw_get_fmt  = icm20608_write_raw_get_fmt,      /* 用户空间写数据 */
};


/*
  * @description    : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - spi  	: spi设备
  * @return  		: 0,成功；其他值，失败
  */
static int icm20608_probe(struct spi_device *spi){
    int ret;
    struct icm20608_dev *dev;
    struct iio_dev *indio_dev;

    /* 1.申请io_dev内存 */
    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev)); /* 这里的 *dev 是解引用，如果使用 dev 的话就是只申请一个指针的地址 
                                                                    而不是 icm20608_dev 结构体的地址*/
    if(!indio_dev){
        return -ENOMEM;
    }
    printk("step1!\r\n");
    /* 2.获取icm20608_dev结构体的地址 */
    dev = iio_priv(indio_dev);
    dev->spi = spi;
    spi_set_drvdata(spi, indio_dev);    /* 将 indio_dev 设置为 spi_dev 的 driver_data */
    mutex_init(&dev->lock);
    printk("step2!\r\n");
    /* 3.iio_dev 的其他成员变量 */
    indio_dev->dev.parent = &spi->dev;
    indio_dev->info = &icm20608_info;
    indio_dev->name = ICM20608_NAME;
    indio_dev->modes = INDIO_DIRECT_MODE;   /* 直接模式，提供 sysfs 接口 */
    indio_dev->channels = icm20608_channels;
    indio_dev->num_channels = ARRAY_SIZE(icm20608_channels);


    /* 4.注册iio_dev */
    ret = iio_device_register(indio_dev);
    if(ret < 0){
        dev_err(&spi->dev, "iio_device_register failed!\r\n");
        goto err_iio_register;
    }
    printk("step4!\r\n");
    /* 5.初始化regmap_config */
    dev->regmap_config.reg_bits = 8;    /* 寄存器地址长度为8 */
    dev->regmap_config.val_bits = 8;   /* 值长度为8 */
    dev->regmap_config.read_flag_mask = 0x80;   /* 读掩码设置为0x80，icm20608使用SPI接口读的时候寄存器的最高位为1 */
    
    /* 6.初始化SPI接口的regmap */
    dev->regmap = regmap_init_spi(spi, &dev->regmap_config);
    if(IS_ERR(dev->regmap)){
        ret = PTR_ERR(dev->regmap);
        goto err_regmap_init;
    }
    printk("step6!\r\n");
    /* 7.初始化spi_device */
    spi->mode = SPI_MODE_0;     /* MODE0, CPOL=0,CPHA=0 */
    spi_setup(spi);
    printk("step7!\r\n");
    /* 初始化ICM20608內部的寄存器 */
    icm20608_reginit(dev);
    return 0;

err_regmap_init:
    iio_device_unregister(indio_dev);
err_iio_register:
    return ret;

}

/*
 * @description     : spi驱动的remove函数，移除spi驱动的时候此函数会执行
 * @param - spi 	: spi设备
 * @return          : 0，成功;其他负值,失败
 */
static int icm20608_remove(struct spi_device *spi){
    struct iio_dev *indio_dev = spi_get_drvdata(spi);
    struct icm20608_dev *dev;
    dev = iio_priv(indio_dev);
    
    regmap_exit(dev->regmap);

    iio_device_unregister(indio_dev);
    return 0;
}


/* 传统的匹配方式ID列表 */
static const struct spi_device_id icm20608_id[] = {
    {"alientek,icm20608", 0},
    {}
};

/* 设备树匹配列表 */
static const struct of_device_id icm20608_of_match[] = {
    {.compatible = "alientek,icm20608"},
    {},
};

static struct spi_driver icm20608_driver = {
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
    return spi_register_driver(&icm20608_driver);
}


static void __exit icm20608_exit(void){
    spi_unregister_driver(&icm20608_driver);
}

module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");