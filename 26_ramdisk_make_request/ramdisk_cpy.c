#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/blkdev.h>
#include<linux/hdreg.h>

#define RAMDISK_SIZE    (2*1024*1024)
#define RAMDISK_NAME    "ramdisk"
#define RAMDISK_MINORS  3

struct ramdisk_dev{
    int major;
    struct gendisk *gendisk;
    struct request_queue *queue;
    unsigned char *ramdiskbuf;
    spinlock_t lock;
};

struct ramdisk_dev ramdisk;

static int ramdisk_open(struct block_device *bdev, fmode_t mode){
	printk("ramdisk_open!!!\r\n");
	return 0;
}

static void ramdisk_release(struct gendisk *disk, fmode_t mode){
	printk("ramdisk_release!!!\r\n");
}

static int ramdisk_getgeo(struct block_device *dev, struct hd_geometry *geo){
	printk("ramdisk_getgeo!!!\r\n");
	/* 硬盘信息 */
	geo->heads = 2;								/* 磁头数 */
	geo->cylinders = 32;						/* 柱面 */
	geo->sectors = RAMDISK_SIZE / (2*32*512);	/* 磁道上的扇区数量 */
	return 0;
}

static const struct block_device_operations ramdisk_fops =
{
	.owner		= THIS_MODULE,
	.open		= ramdisk_open,
	.release	= ramdisk_release,
	.getgeo		= ramdisk_getgeo,
};

/* 制造请求函数 */
static void ramdisk_make_request(struct request_queue *queue, struct bio *bio){
    int offset;
    struct bio_vec bvec;
    struct bvec_iter iter;
    unsigned long len = 0;
    offset = bio->bi_iter.bi_sector << 9;
    bio_for_each_segment(bvec, bio, iter){
        char *ptr = page_address(bvec.bv_page) + bvec.bv_offset;
        len = bvec.bv_len;
        if(bio_data_dir(bio) == READ){
            memcpy(ptr, ramdisk.ramdiskbuf + offset, len);
        }
        else
            memcpy(ramdisk.ramdiskbuf + offset, ptr, len);
    }

    set_bit(BIO_UPTODATE, &bio->bi_flags);
    bio_endio(bio, 0);
}

static int __init ramdisk_init(void){
    int ret = 0;

    ramdisk.ramdiskbuf = kzalloc(RAMDISK_SIZE, GFP_KERNEL);
    if(!ramdisk.ramdiskbuf){
        ret = -EINVAL;
        goto fail_kzalloc;
    }

    ramdisk.major = register_blkdev(0, RAMDISK_NAME);
    if(ramdisk.major < 0){
        ret = -EINVAL;
        goto fail_register_blkdev;
    }
    printk("ramdisk major=%d\r\n", ramdisk.major);

    ramdisk.gendisk = alloc_disk(RAMDISK_MINORS);
    if(!ramdisk.gendisk){
        ret = -EINVAL;
        goto fail_alloc_disk;
    }

    spin_lock_init(&ramdisk.lock);
    /* 分配请求队列 */
    ramdisk.queue = blk_alloc_queue(GFP_KERNEL);
    if(!ramdisk.queue){
        ret = -EINVAL;
        goto out_queue;
    }

    /* 设置制造请求函数 */
    blk_queue_make_request(ramdisk.queue, ramdisk_make_request);

    ramdisk.gendisk->major = ramdisk.major;
    ramdisk.gendisk->first_minor = 0;
    ramdisk.gendisk->fops = &ramdisk_fops;
    ramdisk.gendisk->private_data = &ramdisk;
    ramdisk.gendisk->queue = ramdisk.queue;
    sprintf(ramdisk.gendisk->disk_name, RAMDISK_NAME);
    set_capacity(ramdisk.gendisk, RAMDISK_SIZE/512);

    add_disk(ramdisk.gendisk);

    return ret;

out_queue:
    put_disk(ramdisk.gendisk);
fail_alloc_disk:
    unregister_blkdev(ramdisk.major, RAMDISK_NAME);
fail_register_blkdev:
    kfree(ramdisk.ramdiskbuf);    
fail_kzalloc:
    return ret;
}

static void __exit ramdisk_exit(void){
    del_gendisk(ramdisk.gendisk);
    put_disk(ramdisk.gendisk);
    blk_cleanup_queue(ramdisk.queue);
    unregister_blkdev(ramdisk.major, RAMDISK_NAME);
    kfree(ramdisk.ramdiskbuf);
}

module_init(ramdisk_init);
module_exit(ramdisk_exit);
MODULE_LICENSE("GFP");
MODULE_AUTHOR("azumid");