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

/* 定义磁盘大小，内存模式 */
#define RAMDISK_SIZE	(2*1024*1024)	/* 大小2M */
#define RAMDISK_NAME 	"ramdisk"
#define RAMDISK_MINOR	3				/* 表示3个分区 */

/* ramdisk设备结构体 */
struct ramdisk_dev{
	int major;	/* 主设备号 */
	unsigned char *ramdiskbuf;	/* ramdisk的内存空间，模拟磁盘空间	 */
	struct gendisk *gendisk;
	struct request_queue *queue;
	spinlock_t lock;	/* 自旋锁 */
};

struct ramdisk_dev ramdisk;

static void ramdisk_transfer(struct request *req){
	/* 数据传输的三要素：源，目的地，长度 。
		内存地址，块设备地址，长度 
	*/
	unsigned long start = blk_rq_pos(req) << 9;		/*  blk_rq_pos获取到要操作块设备的扇区地址,因为这个函数获取到的是块，
														一个块的大小是521字节，左移9位，地址字节 */

	unsigned long len  = blk_rq_cur_bytes(req);		/* 数据长度 */

	/**
	 * 获取bio里面的缓冲区
	 * 如果是读：从磁盘里读取到的数据保存在此缓冲区中
	 * 如果是写：此缓冲区保存着要写入到磁盘里的数据
	 */
	void *buffer = bio_data(req->bio);
	if (rq_data_dir(req) == READ)			/* 读操作 */			
		memcpy(buffer, ramdisk.ramdiskbuf + start, len);
	else									/* 写操作 */
		memcpy(ramdisk.ramdiskbuf + start, buffer, len);

}

/* 请求函数 */
static void ramdisk_request_fn(struct request_queue *q){
	struct request *req;
	int err = 0;
	req = blk_fetch_request(q);	/* 有电梯调度算法 */
	while (req) {
		/* 处理request，也就是具体数据的读写操作 */
		ramdisk_transfer(req);
		if (!__blk_end_request_cur(req, err))
		req = blk_fetch_request(q);
	}
}

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

/* 块设备操作集 */
static const struct block_device_operations ramdisk_fops =
{
	.owner		= THIS_MODULE,
	.open		= ramdisk_open,
	.release	= ramdisk_release,
	.getgeo		= ramdisk_getgeo,
};

/* 驱动入口函数 */
static int __init ramdisk_init(void){
	int ret = 0;
	printk("ramdisk_init!!!\r\n");

	/* 1.先申请内存 */
	ramdisk.ramdiskbuf = kzalloc(RAMDISK_SIZE, GFP_KERNEL);
	if(ramdisk.ramdiskbuf == NULL){
		ret = -EINVAL;
		goto fail_kzalloc;
	}

	/* 2.注册块设备 */
	ramdisk.major = register_blkdev(0, RAMDISK_NAME);	/* 0表示系统自动分配主设备号 */
	if(ramdisk.major < 0){
		ret = -EINVAL;
		goto ramdisk_register_blkdev_fail;
	}
	printk("ramdisk major=%3d\r\n", ramdisk.major);

	/* 3.申请gendisk */
	ramdisk.gendisk = alloc_disk(RAMDISK_MINOR);
	if(!ramdisk.gendisk){
		ret = -EINVAL;
		goto out_disk;
	}

	/* 4.初始化自旋锁 */
	spin_lock_init(&ramdisk.lock);

	/* 5、分配并初始化请求队列 */
	ramdisk.queue = blk_init_queue(ramdisk_request_fn, &ramdisk.lock);
	if(!ramdisk.queue) {
		ret = EINVAL;
		goto out_queue;
	}

	
	/* 6.初始化gendisk */
	ramdisk.gendisk->major = ramdisk.major;
    ramdisk.gendisk->first_minor = 0;
    ramdisk.gendisk->fops = &ramdisk_fops;
	ramdisk.gendisk->private_data = &ramdisk;
	ramdisk.gendisk->queue = ramdisk.queue;
    sprintf(ramdisk.gendisk->disk_name, RAMDISK_NAME);
	set_capacity(ramdisk.gendisk, RAMDISK_SIZE/512);	/* 设置容量，也就是设置扇区的数量 */
    
    add_disk(ramdisk.gendisk);	/* 添加到内核中 */

	return 0;

out_queue:
	put_disk(ramdisk.gendisk);
out_disk:
	unregister_blkdev(ramdisk.major, RAMDISK_NAME);
ramdisk_register_blkdev_fail:
	kfree(ramdisk.ramdiskbuf);	/* 释放申请的内存 */
fail_kzalloc:
	return ret;

}

/* 驱动出口函数 */
static void __exit ramdisk_exit(void){
	printk("ramdisk_exit!!!\r\n");
	del_gendisk(ramdisk.gendisk);
	put_disk(ramdisk.gendisk);

	blk_cleanup_queue(ramdisk.queue);	/* 清除队列 */
	unregister_blkdev(ramdisk.major, RAMDISK_NAME);
	kfree(ramdisk.ramdiskbuf);
}

module_init(ramdisk_init);
module_exit(ramdisk_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");


