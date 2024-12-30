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

struct icm20608_dev {
    struct spi_device *spi;
    struct regmap *regmap;
    struct regmap_config regmap_config;
    struct mutex lock;
};



static int icm20608_probe(struct spi_device *spi){
    int ret = 0;
    struct iio_device *indio_dev;
    struct icm20608_dev *dev;
    
    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
    if(!indio_dev) return -ENOMEM;
    
    dev = iio_priv(indio_dev);
    dev->spi = spi;
    spi_set_drvdata(spi, indio_dev);
    mutex_init(&dev->lock);

    indio_dev->dev.parent = &spi->dev;
    indio_dev->info = ;
    indio_dev->name = ICM20608_NAME;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = ;
    indio_dev->num_channels = ;


    return ret;
}

static int icm20608_remove(struct spi_device *spi){
    int ret = 0;
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