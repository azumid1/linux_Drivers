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
    unsigned char *ramdiskbuf;
    struct request_queue *queue;
    spinlock_t lock;
};

struct ramdisk_dev ramdisk;


static int ramdisk_open(struct block_device *bdev, fmode_t mode){
    return 0;
}

static void ramdisk_release(struct gendisk *disk, fmode_t mode){

}

static const struct block_device_operations ramdisk_fops =
{
	.owner		= THIS_MODULE,
	.open		= ramdisk_open,
	.release	= ramdisk_release,
};


static void ramdisk_transfer(struct request *req){
    unsigned long start = blk_rq_pos(req) << 9;
    unsigned long len = blk_rq_cur_bytes(req);

    void *buffer = bio_data(req->bio);
    if(rq_data_dir(req) == READ)
        memcpy(buffer, ramdisk.ramdiskbuf + start, len);
    else
        memcpy(ramdisk.ramdiskbuf + start, buffer, len);
}

/* 请求函数 */
static void ramdisk_request_fn(struct request_queue *q){
    struct request *req;
    int err = 0;
    req = blk_fetch_request(q);
    while(req){
        ramdisk_transfer(req);

        /* 这里是判断一下是不是最后一个req */
        if (!__blk_end_request_cur(req, err))
		req = blk_fetch_request(q);
    }
}

static int __init ramdisk_init(void){
    int ret = 0;
    /* 1.先申请内存 */
    ramdisk.ramdiskbuf = kzalloc(RAMDISK_SIZE, GFP_KERNEL);
    if(ramdisk.ramdiskbuf == NULL){
        ret = -EINVAL;
        goto fail_kzalloc;
    }

    /* 2.注册块设备 */
    ramdisk.major = register_blkdev(0, RAMDISK_NAME);
    if(ramdisk.major < 0){
        ret = -EINVAL;
        goto fail_ramdisk_register_blkdev;
    }
    printk("ramdisk major=%d\r\n", ramdisk.major);

    /* 3.申请gendisk */
    ramdisk.gendisk = alloc_disk(RAMDISK_MINORS);
    if(!ramdisk.gendisk){
        ret = -EINVAL;
        goto fail_alloc_disk;
    }

    /* 4.初始化自旋锁 */
    spin_lock_init(&ramdisk.lock);
    
    /* 5.分配并初始化请求队列 */
    ramdisk.queue = blk_init_queue(ramdisk_request_fn, &ramdisk.lock);
    if(!ramdisk.queue){
        ret = -EINVAL;
        goto out_queue;
    }

    /* 6.初始化gendisk */
    ramdisk.gendisk->major = ramdisk.major;
    ramdisk.gendisk->first_minor = 0;
    ramdisk.gendisk->fops = &ramdisk_fops;
    ramdisk.gendisk->queue = ramdisk.queue;
    sprintf(ramdisk.gendisk->disk_name, RAMDISK_NAME);

    set_capacity(ramdisk.gendisk, RAMDISK_SIZE/512);
    add_disk(ramdisk.gendisk);

    return 0;

out_queue:
    put_disk(ramdisk.gendisk);
fail_alloc_disk:
    unregister_blkdev(ramdisk.major, RAMDISK_NAME);
fail_ramdisk_register_blkdev:
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
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");