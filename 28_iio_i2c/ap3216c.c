#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/unaligned/be_byteshift.h>
#include <linux/iio/trigger.h>
#include "ap3216creg.h"

#define AP3216C_NAME    "ap3216c"

/* 
 * AP3216C的扫描元素，1路ALS(环境关)，1路PS(距离传感器)，1路IR
 */
enum imv_ap3216c_scan {
    AP3216C_ALS,
	AP3216C_PS,
	AP3216C_IR,
};

/* 
 * ap3216c环境光传感器分辨率,扩大1000000倍,
 * 量程依次为0～20661，0～5162，0～1291，0～323。单位：lux
 */
static const int als_scale_ap3216c[] = {315000, 78800, 19700, 4900};

struct ap3216c_dev {
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_config regmap_config;
    struct mutex lock;
    struct iio_trigger *trig;
};

/*
 * 扫描掩码，两种情况，全启动0X111，或者都不启动0X0
 */
static const unsigned long ap3216c_scan_masks[] = {
	BIT(AP3216C_ALS)
	| BIT(AP3216C_PS)
	| BIT(AP3216C_IR),
	0,
};

/*
 * ap3216c通道，1路ALS(环境关)，1路PS(距离传感器)，1路IR
 */
static const struct iio_chan_spec ap3216c_channels[] = {
    /* ALS通道 */
    {
        .type = IIO_INTENSITY,
        .modified = 1,
        .channel2 = IIO_MOD_LIGHT_BOTH,
        .address = AP3216C_ALSDATALOW,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
            BIT(IIO_CHAN_INFO_SCALE),
        .scan_index = AP3216C_ALS,
        .scan_type = {
            .sign = 'u',
            .realbits = 16,
            .storagebits = 16,
            .endianness = IIO_LE,
        },
    },

    /* PS通道 */
    {
        .type = IIO_PROXIMITY,
        .address = AP3216C_PSDATALOW,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .scan_index = AP3216C_PS,
        .scan_type = {
            .sign = 'u',
            .realbits = 10,
            .storagebits = 16,
            .endianness = IIO_LE,
        },
    },

    /* IR通道 */
    {
        .type = IIO_INTENSITY,
        .modified = 1,
        .channel2 = IIO_MOD_LIGHT_IR,
        .address = AP3216C_IRDATALOW,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .scan_index = AP3216C_IR,
        .scan_type = {
            .sign = 'u',
            .realbits = 10,
            .storagebits = 16,
            .endianness = IIO_LE,
        },
    },

};

/*
  * @description  	: 读取AP3216C传感器数
  * @param - dev	: ap3216c设备 
  * @param - reg  	: 要读取的通道寄存器首地址。
  * @param - chann2 : 需要读取的通道，比如ALS，IR。
  * @param - val  	: 保存读取到的值。
  * @return			: 0，成功；其他值，错误
  */
 static int ap3216c_read_alsir_data(struct ap3216c_dev *dev, int reg,
				   int chann2, int *val)
{
    int ret = 0;
    unsigned char data[2];
    switch(chann2){
        case IIO_MOD_LIGHT_BOTH:
            ret = regmap_bulk_read(dev->regmap, reg, data, 2);
            *val = ((int)data[1] << 8) | data[0];
            break;
        case IIO_MOD_LIGHT_IR:
            ret = regmap_bulk_read(dev->regmap, reg, data, 2);
            *val = ((int)data[1] << 2) | (data[0] & 0x03);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    if(ret){
        return -EINVAL;
    }
    return IIO_VAL_INT;
}

/*
  * @description  	: 设置AP3216C的ALS量程(分辨率)
  * @param - dev	: ap3216c设备
  * @param - val   	: 量程(分辨率值)。
  * @param - chann2 : 需要设置的通道。
  * @return			: 0，成功；其他值，错误
  */
static int ap3216c_write_als_scale(struct ap3216c_dev *dev, int chann2, int val){
    int ret = 0, i;
    u8 d;
    switch(chann2){
        case IIO_CHAN_INFO_SCALE:
            for(i=0; i<ARRAY_SIZE(als_scale_ap3216c); i++){
                if(als_scale_ap3216c[i] == val){
                    d = (i << 4);
                    ret = regmap_write(dev->regmap, AP3216C_ALSCONFIG, d);
                }
            }
            break;
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

static unsigned char ap3216c_read_reg(struct ap3216c_dev *dev, u8 reg)
{
	u8 ret;
	unsigned int data;

	ret = regmap_read(dev->regmap, reg, &data);
	return (u8)data;
}


static void ap3216c_write_reg(struct ap3216c_dev *dev, u8 reg, u8 data){
    regmap_write(dev->regmap, reg, data);
}


/*
  * @description     	: 读函数，当读取sysfs中的文件的时候最终此函数会执行，此函数
  * 					：里面会从传感器里面读取各种数据，然后上传给应用。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - val   		: 读取的值，如果是小数值的话，val是整数部分。
  * @param - val2   	: 读取的值，如果是小数值的话，val2是小数部分。
  * @return				: 0，成功；其他值，错误
  */

static int ap3216c_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret = 0;
	unsigned char data[2];
	unsigned char regdata;
	struct ap3216c_dev *dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:								/* 读取ICM20608加速度计、陀螺仪、温度传感器原始值 */
		mutex_lock(&dev->lock);								/* 上锁 			*/
		switch (chan->type) {
		case IIO_INTENSITY:
			ret = ap3216c_read_alsir_data(dev, chan->address, chan->channel2, val); /* 读取ALS */
			break;				/* 值为val */
		case IIO_PROXIMITY:
			ret = regmap_bulk_read(dev->regmap, chan->address, data, 2);
			*val = ((int)(data[1] & 0X3F) << 4) | (data[0] & 0X0F);  
			ret = IIO_VAL_INT; 	/* 值为val */
			break;
		default:
			ret = -EINVAL;
			break;
		}
		mutex_unlock(&dev->lock);							/* 释放锁 			*/
		return ret;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_INTENSITY:			/* ALS量程 */
			mutex_lock(&dev->lock);
            regdata = ap3216c_read_reg(dev, AP3216C_ALSCONFIG);
			regdata = (( regdata & 0X30 ) >> 4);
			*val  = 0;
			*val2 = als_scale_ap3216c[regdata];
			mutex_unlock(&dev->lock);
			return IIO_VAL_INT_PLUS_MICRO;	/* 值为val+val2/1000000 */
		default:
			return -EINVAL;
		}
		return ret;
		
	default:
		return -EINVAL;
	}
	return ret;
}

 /* @description     	: 写函数，当向sysfs中的文件写数据的时候最终此函数会执行，一般在此函数
  * 					：里面设置传感器，比如量程等。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - val   		: 应用程序写入的值，如果是小数值的话，val是整数部分。
  * @param - val2   	: 应用程序写入的值，如果是小数值的话，val2是小数部分。
  * @return				: 0，成功；其他值，错误
  */
static int ap3216c_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	int ret = 0;
	struct ap3216c_dev *dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:	/* 设置ALS量程 */
		switch (chan->type) {
		case IIO_INTENSITY:		/* 设置ALS量程 */
			mutex_lock(&dev->lock);
			ret = ap3216c_write_als_scale(dev, chan->channel2, val2);
			mutex_unlock(&dev->lock);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
  * @description     	: 用户空间写数据格式，比如我们在用户空间操作sysfs来设置传感器的分辨率，
  * 					：如果分辨率带小数，那么这个小数传递到内核空间应该扩大多少倍，此函数就是
  *						: 用来设置这个的。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - mask   	: 掩码
  * @return				: 0，成功；其他值，错误
  */
static int ap3216c_write_raw_get_fmt(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, long mask){
    switch(mask){
        case IIO_CHAN_INFO_SCALE:
            switch(chan->type){
                case IIO_INTENSITY:
                    return IIO_VAL_INT_PLUS_MICRO;
                default:
                    return IIO_VAL_INT_PLUS_MICRO;
            }
        default:
            return IIO_VAL_INT_PLUS_MICRO;
    }
    return -EINVAL;
}


/*
 * iio_info结构体变量
 */
static const struct iio_info ap3216c_info = {
    .read_raw = ap3216c_read_raw,
    .write_raw = ap3216c_write_raw,
    .write_raw_get_fmt = &ap3216c_write_raw_get_fmt,    /* 用户空间写数据 */
};


static int ap3216c_reginit(struct ap3216c_dev *dev){
    /* 初始化AP3216C */
    ap3216c_write_reg(dev, AP3216C_SYSTEMCONG, 0x04);   /* 先软复位 */
    mdelay(50);
    ap3216c_write_reg(dev, AP3216C_SYSTEMCONG, 0x03);   /* 开启ALS,PS,IR */
    ap3216c_write_reg(dev, AP3216C_ALSCONFIG, 0x00);    /* ALS单次转换触发，量程为0-20661 lux */
    ap3216c_write_reg(dev, AP3216C_PSLEDCONFIG, 0x13);  /* IR LED 1脉冲，驱动电流100%，这不就是默认值啊 */
    return 0;
}

static int ap3216c_probe(struct i2c_client *client, const struct i2c_device_id *id){
    int ret = 0;
    struct iio_dev *indio_dev;
    struct ap3216c_dev *dev;

    /* 1.申请内存 */
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*dev));
    if(!indio_dev) return -ENOMEM;

    /* 2.获取ap3216c_dev 的地址 */
    dev = iio_priv(indio_dev);
    dev->client = client;

    i2c_set_clientdata(client, indio_dev);   /* 将 indio_dev 设置为 client 的 数据 */
	mutex_init(&dev->lock);	

    /* iio_dev的其他成员变量 */
    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &ap3216c_info;
    indio_dev->name = AP3216C_NAME;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = ap3216c_channels;
    indio_dev->num_channels = ARRAY_SIZE(ap3216c_channels);
    indio_dev->available_scan_masks = ap3216c_scan_masks;

    /* 注册iio_dev */
    ret = iio_device_register(indio_dev);
    if(ret < 0){
        dev_err(&client->dev, "iio_device_register failed!\r\n");
        goto err_iio_register;
    }

    /* 初始化regmap的配置 */
    dev->regmap_config.reg_bits = 8;
    dev->regmap_config.val_bits = 8;

    /* 初始化IIC接口的regmap */
    dev->regmap = regmap_init_i2c(client, &dev->regmap_config);
	if (IS_ERR(dev->regmap)) {
		ret = PTR_ERR(dev->regmap);
		goto err_regmap_init;
	}	

    ap3216c_reginit(dev);

    return 0;

err_regmap_init:
    iio_device_unregister(indio_dev);
err_iio_register:
    return ret;
}

static int ap3216c_remove(struct i2c_client *client){
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct ap3216c_dev *dev = iio_priv(indio_dev);

    /* 释放regmap */
    regmap_exit(dev->regmap);
    
    iio_device_unregister(indio_dev);
    return 0;
}

static const struct i2c_device_id ap3216c_id[] = {
    {"alientek,ap3216c", 0},
    {}
};

static const struct of_device_id ap3216c_of_match[] = {
    {.compatible = "alientek,ap3216c"},
    {}
};

static struct i2c_driver ap3216c_driver = {
    .probe = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "ap3216c",
        .of_match_table = ap3216c_of_match,
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
