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
#define ICM20608_TEMP_SCALE     326800000
#define ICM20608_TEMP_OFFSET    0

#define ICM20608_CHAN(_type, _channel2, _index)  \
    {                                           \
        .type = _type,                          \
        .modified = 1,                          \
        .channel2 = _channel2,                  \
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),   \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW)|   \
                                BIT(IIO_CHAN_INFO_CALIBBIAS), \
        .scan_index = _index,                   \
        .scan_type = {                          \
            .sign = 'u',                        \
            .realbits = 16,                     \
            .storagebits = 16,                  \
            .shift = 0,                         \
            .endianness = IIO_BE,               \
        },                                      \
    }

enum inv_icm20608_scan{
    INV_ICM20608_SCAN_ACCL_X,
	INV_ICM20608_SCAN_ACCL_Y,
	INV_ICM20608_SCAN_ACCL_Z,
	INV_ICM20608_SCAN_TEMP,
	INV_ICM20608_SCAN_GYRO_X,
	INV_ICM20608_SCAN_GYRO_Y,
	INV_ICM20608_SCAN_GYRO_Z,
	INV_ICM20608_SCAN_TIMESTAMP,
};

/*
 * icm20608陀螺仪分辨率，对应250、500、1000、2000，计算方法：
 * 以正负250度量程为例，500/2^16=0.007629，扩大1000000倍，就是7629
 */
static const int gyro_scale_icm20608[] = {7629, 15258, 30517, 61035};

/* 
 * icm20608加速度计分辨率，对应2、4、8、16 计算方法：
 * 以正负2g量程为例，4/2^16=0.000061035，扩大1000000000倍，就是61035
 */
static const int accel_scale_icm20608[] = {61035, 122070, 244140, 488281};

static const struct iio_chan_spec icm20608_chans[] = {
    /* temp */
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
                BIT(IIO_CHAN_INFO_OFFSET) |
                BIT(IIO_CHAN_INFO_SCALE),
        .scan_index = INV_ICM20608_SCAN_TEMP,
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_BE,
        },
    },

    ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_ICM20608_SCAN_GYRO_X),
    ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_ICM20608_SCAN_GYRO_Y),
    ICM20608_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_ICM20608_SCAN_GYRO_Z),

    ICM20608_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_ICM20608_SCAN_ACCL_Y),	/* 加速度X轴 */
	ICM20608_CHAN(IIO_ACCEL, IIO_MOD_X, INV_ICM20608_SCAN_ACCL_X),	/* 加速度Y轴 */
	ICM20608_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_ICM20608_SCAN_ACCL_Z),	/* 加速度Z轴 */
};

struct icm20608_dev {
    struct spi_device *spi;
    struct regmap *regmap;
    struct regmap_config regmap_config;
    struct mutex lock;
};

static void icm20608_write_reg(struct icm20608_dev *dev, u8 reg, u8 value){
    regmap_write(dev->regmap, reg, value);
}

static unsigned char icm20608_read_reg(struct icm20608_dev *dev, u8 reg){
    unsigned int data;
    u8 ret;
    ret = regmap_read(dev->regmap, reg, &data);
    return (u8)data;
}

static int icm20608_sensor_show(struct icm20608_dev *dev, int reg, int axis, int *val){
    int ind, ret;
    __be16 d;
    ind = (axis - IIO_MOD_X) * 2;
    ret = regmap_bulk_read(dev->regmap, reg, (u8 *)&d, 2);
    if(ret){
        return -EINVAL;
    }
    *val = (short)be16_to_cpup(&d);
    return IIO_VAL_INT;
}

static int icm20608_sensor_set(struct icm20608_dev *dev, int reg,
            int axis, int val)
{
    int ind, result;
    __be16 d = cpu_to_be16(val);
    
    ind = (axis - IIO_MOD_X) * 2;
    result = regmap_bulk_write(dev->regmap, reg + ind, (u8 *)&d, 2);
    if(result)
        return -EINVAL;

    return 0;
}

static int icm20608_read_channel_data(struct iio_dev *indio_dev, struct iio_chan_spec const * chan,
                                        int *val)
{
    struct icm20608_dev *dev = iio_priv(indio_dev);
    int ret = 0;
    switch(chan->type){
        case IIO_ANGL_VEL:
            ret = icm20608_sensor_show(dev, ICM20_GYRO_XOUT_H, chan->channel2, val);
            break;
        case IIO_ACCEL:
            ret = icm20608_sensor_show(dev, ICM20_ACCEL_XOUT_H, chan->channel2, val);
            break;
        case IIO_TEMP:
            ret = icm20608_sensor_show(dev, ICM20_TEMP_OUT_H, chan->channel2, val);
            break;
        default:
            ret = -EINVAL;
            break;        
    }
    return ret;
}

static int icm20608_write_gyro_scale(struct icm20608_dev *dev, int val)
{
    int ret,i;
    u8 d;
    for(i=0; i< ARRAY_SIZE(gyro_scale_icm20608); i++){
        if(gyro_scale_icm20608[i] == val){
            d = (i << 3);
            ret = regmap_write(dev->regmap, ICM20_GYRO_CONFIG, d);
            if(ret) return ret;
            return 0;
        }
    }
    return -EINVAL;
}

static int icm20608_write_accel_scale(struct icm20608_dev *dev, int val)
{
    int ret,i;
    u8 d;
    for(i=0; i< ARRAY_SIZE(accel_scale_icm20608); i++){
        if(accel_scale_icm20608[i] == val){
            d = (i << 3);
            ret = regmap_write(dev->regmap, ICM20_ACCEL_CONFIG, d);
            if(ret) return ret;
            return 0;
        }
    }
    return -EINVAL;
}

static int icm20608_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
    struct icm20608_dev *dev = iio_priv(indio_dev);
    int ret = 0;
    unsigned char regdata = 0;
    switch(mask){
        case IIO_CHAN_INFO_SCALE:
            switch(chan->type){
                case IIO_ANGL_VEL:
                    mutex_lock(&dev->lock);
                    regdata = (icm20608_read_reg(dev, ICM20_GYRO_CONFIG) & 0x18) >> 3;
                    *val = 0;
                    *val2 = gyro_scale_icm20608[regdata];
                    mutex_unlock(&dev->lock);
                    return IIO_VAL_INT_PLUS_MICRO;
                case IIO_ACCEL:
                    mutex_lock(&dev->lock);
                    regdata = (icm20608_read_reg(dev, ICM20_ACCEL_CONFIG) & 0x18) >> 3;
                    *val = 0;
                    *val2 = accel_scale_icm20608[regdata];
                    mutex_unlock(&dev->lock);
                    return IIO_VAL_INT_PLUS_MICRO;
                case IIO_TEMP:
                /* 解释一下温度的量程为什么是326800000，这个文档中没有，但是教程里就是这么写的 */
                    *val = ICM20608_TEMP_SCALE / 1000000;
                    *val2 = ICM20608_TEMP_SCALE % 1000000;
                    return IIO_VAL_INT_PLUS_MICRO;
                default:
                    return -EINVAL;
            }
        case IIO_CHAN_INFO_RAW:
            mutex_lock(&dev->lock);
            ret = icm20608_read_channel_data(indio_dev, chan, val);
            mutex_unlock(&dev->lock);
            return ret;
        case IIO_CHAN_INFO_OFFSET:
            switch(chan->type){
                case IIO_TEMP:
                    *val = ICM20608_TEMP_OFFSET;
                    return IIO_VAL_INT;
                default:
                    return -EINVAL;
            }
        case IIO_CHAN_INFO_CALIBBIAS:
            switch(chan->type){
                case IIO_ANGL_VEL:
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
    struct icm20608_dev *dev = iio_priv(indio_dev);
    int ret = 0;
    switch(mask){
        case IIO_CHAN_INFO_SCALE:
            switch(chan->type){
                case IIO_ANGL_VEL:
                    mutex_lock(&dev->lock);
                    ret = icm20608_write_gyro_scale(dev, val);
                    mutex_unlock(&dev->lock);
                    break;
                case IIO_ACCEL:
                    mutex_lock(&dev->lock);
                    ret = icm20608_write_accel_scale(dev, val);
                    mutex_unlock(&dev->lock);
                    break;
                default:
                    ret = -EINVAL;
                    break;
            }
            break;
        case IIO_CHAN_INFO_CALIBBIAS:
            switch(chan->type){
                case IIO_ANGL_VEL:
                    mutex_lock(&dev->lock);
                    ret = icm20608_sensor_set(dev, ICM20_XG_OFFS_USRH,
                                                chan->channel2, val);
                    mutex_unlock(&dev->lock);
                    break;
                case IIO_ACCEL:
                    mutex_lock(&dev->lock);
                    ret = icm20608_sensor_set(dev, ICM20_XA_OFFSET_H,
                                                chan->channel2, val);
                    mutex_unlock(&dev->lock);
                    break;
                default:
                    return -EINVAL;    
            }
            break;
        default:
            return -EINVAL;
    }
    return ret;
}

static int icm20608_write_raw_get_fmt(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, long mask)
{
    switch(mask){
        case IIO_CHAN_INFO_SCALE:
            switch(chan->type){
                case IIO_ANGL_VEL:
                    return IIO_VAL_INT_PLUS_MICRO;
                default:
                    return IIO_VAL_INT_PLUS_NANO;
            }
        default:
            return IIO_VAL_INT_PLUS_MICRO;
    }
    return -EINVAL;
}

static const struct iio_info icm20608_info = {
    .read_raw = icm20608_read_raw,
    .write_raw = icm20608_write_raw,
    .write_raw_get_fmt = icm20608_write_raw_get_fmt, 
};



void icm20608_reginit(struct icm20608_dev *dev){
    u8 ret = 0;
    /* reset */
    icm20608_write_reg(dev, ICM20_PWR_MGMT_1, 0x80);
    mdelay(50);
    icm20608_write_reg(dev, ICM20_PWR_MGMT_1, 0x01);
    mdelay(50);

    ret = icm20608_read_reg(dev, ICM20_PWR_MGMT_1);
    printk("ICM20608 ID = %#x\r\n", ret);
    
    icm20608_write_reg(dev, ICM20_SMPLRT_DIV, 0x00); 
    icm20608_write_reg(dev, ICM20_GYRO_CONFIG, 0x18);
    icm20608_write_reg(dev, ICM20_ACCEL_CONFIG, 0x18);
    icm20608_write_reg(dev, ICM20_CONFIG, 0x04);
    /* 解释一下下面的这个配置
        因为ICM20_GYRO_CONFIG的FCHOICE_B被配置为了00
        所以可以通过DLPF_CFG来配置DLPF，查表即可按表配置
     */ 
    icm20608_write_reg(dev, ICM20_ACCEL_CONFIG2, 0x04); /* 加速度计低通滤波BW=21.2Hz 	*/
	icm20608_write_reg(dev, ICM20_PWR_MGMT_2, 0x00); 	/* 打开加速度计和陀螺仪所有轴 	*/
	icm20608_write_reg(dev, ICM20_LP_MODE_CFG, 0x00); 	/* 关闭低功耗 				*/
	icm20608_write_reg(dev, ICM20_INT_ENABLE, 0x01);

}

static int icm20608_probe(struct spi_device *spi){
    int ret = 0;
    struct iio_dev *indio_dev;
    struct icm20608_dev *dev;
    
    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
    if(!indio_dev) return -ENOMEM;
    
    dev = iio_priv(indio_dev);
    dev->spi = spi;
    spi_set_drvdata(spi, indio_dev);
    mutex_init(&dev->lock);

    indio_dev->dev.parent = &spi->dev;
    indio_dev->info = &icm20608_info;
    indio_dev->name = ICM20608_NAME;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = icm20608_chans;
    indio_dev->num_channels = ARRAY_SIZE(icm20608_chans);

    ret = iio_device_register(indio_dev);
    if(ret < 0){
        dev_err(&spi->dev, "iio_device_register failed!\r\n");
        goto err_iio_register;
    }

    dev->regmap_config.reg_bits = 8;
    dev->regmap_config.val_bits = 8;
    dev->regmap_config.read_flag_mask = 0x80;

    dev->regmap = regmap_init_spi(spi, &dev->regmap_config);
    if(IS_ERR(dev->regmap)){
        ret = PTR_ERR(dev->regmap);
        goto err_regmap_init;
    }

    spi->mode = SPI_MODE_0;
    spi_setup(spi);

    icm20608_reginit(dev);

    return ret;
err_regmap_init:
    iio_device_unregister(indio_dev);
err_iio_register:
    return ret;

}

static int icm20608_remove(struct spi_device *spi){
    int ret = 0;
    struct iio_dev *indio_dev = spi_get_drvdata(spi);
    struct icm20608_dev *dev;
    dev = iio_priv(indio_dev);
    regmap_exit(dev->regmap);
    iio_device_unregister(indio_dev);

    return ret;
}

static const struct spi_device_id icm20608_id[] = {
    {"alientek,icm20608", 0},
    {}
};

static const struct of_device_id icm20608_of_match[] = {
    {.compatible = "alientek,icm20608"},
    {}
};

static struct spi_driver icm20608_driver = {
    .probe = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = ICM20608_NAME,
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