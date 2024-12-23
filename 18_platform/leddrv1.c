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
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: leddriver.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: platform驱动
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/8/13 左忠凯创建
***************************************************************/

#define PLATFORM_CNT		1
#define PLATFORM_NAME		"platled1"
#define LEDOFF				0
#define LEDON				1

/* 映射后的寄存器虚拟地址指针 */
static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

/* 设备结构体	*/
struct newchrled_dev{
	dev_t devid;
	int major;
	int minor;
	struct class *class;
	struct cdev cdev;
	struct device *device;
};

struct newchrled_dev newchrled;	/* led设备 */

#if 0
struct platform_driver {
	int (*probe)(struct platform_device *);
	int (*remove)(struct platform_device *);
	void (*shutdown)(struct platform_device *);
	int (*suspend)(struct platform_device *, pm_message_t state);
	int (*resume)(struct platform_device *);
	struct device_driver driver;
	const struct platform_device_id *id_table;
	bool prevent_deferred_probe;
};

#endif

/*
* @description		: LED打开/关闭
* @param - sta 	: LEDON(0) 打开LED，LEDOFF(1) 关闭LED
* @return 			: 无
*/
void led_switch(u8 sta){
	u32 val = 0;
	if(sta == LEDON){
		val = readl(GPIO1_DR);
		val &= ~(1 << 3);
		writel(val, GPIO1_DR);
	} else if(sta == LEDOFF){
		val = readl(GPIO1_DR);
		val |= (1 << 3);
		writel(val,GPIO1_DR);
	}
}

static int led_open(struct inode *inode, struct file *filp){
	filp->private_data = &newchrled;
	return 0;
}

/*
* @description		: 从设备读取数据 
* @param - filp 	: 要打开的设备文件(文件描述符)
* @param - buf 	: 返回给用户空间的数据缓冲区
* @param - cnt 	: 要读取的数据长度
* @param - offt 	: 相对于文件首地址的偏移
* @return 			: 读取的字节数，如果为负值，表示读取失败
*/

static ssize_t led_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt){
	return 0;
}

/*
* @description		: 向设备写数据 
* @param - filp 	: 设备文件，表示打开的文件描述符
* @param - buf 	: 要写给设备写入的数据
* @param - cnt 	: 要写入的数据长度
* @param - offt 	: 相对于文件首地址的偏移
* @return 			: 写入的字节数，如果为负值，表示写入失败
*/
static ssize_t led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue;
    unsigned char databuf[1];
    unsigned char ledstat;

    retvalue = copy_from_user(databuf, buf, cnt);
    if(retvalue < 0) {
        printk("kernel write failed!\r\n");
        return -EFAULT;
    }

    ledstat = databuf[0];		/* 获取状态值 */

    if(ledstat == LEDON) {	
        led_switch(LEDON);		/* 打开LED灯 */
    } else if(ledstat == LEDOFF) {
        led_switch(LEDOFF);	/* 关闭LED灯 */
    }
    return 0;
}
/*
* @description		: 关闭/释放设备
* @param - filp 	: 要关闭的设备文件(文件描述符)
* @return 			: 0 成功;其他 失败
*/
static int led_release(struct inode *inode, struct file *filp){
	return 0;
}

/* 设备操作函数 */
static struct file_operations platform_fops = {
	.owner = THIS_MODULE,
	.open = led_open,
	.read = led_read,
	.write = led_write,
	.release = led_release,
};
static int led_probe(struct platform_device *dev){
	u32 val  = 0;
	unsigned char i = 0;
	struct resource *ledresource[5];
	/* 初始化LED,字符设备驱动 */
    /* 1,从设备中获取资源 */
    /*
    * platform_get_resource - get a resource for a device
    * @dev: platform device
    * @type: resource type
    * @num: resource index
    */
   	for(i=0; i<5; i++){
		ledresource[i] = platform_get_resource(dev, IORESOURCE_MEM, i);
		if(ledresource[i] == NULL) return -EINVAL;
	}
	/* 这一步是读取了led这个设备所需要的寄存器地址，读出来之后就可以使用了 */
	
	/* 初始化LED */
	/* 1、寄存器地址映射 */
	/* 要注意，ledresource[]是一个指针数组，读出来的数据是一个个指针 */
	IMX6U_CCM_CCGR1 = ioremap(ledresource[0]->start, resource_size(ledresource[0]));
	SW_MUX_GPIO1_IO03 = ioremap(ledresource[1]->start, resource_size(ledresource[1]));
	SW_PAD_GPIO1_IO03 = ioremap(ledresource[2]->start, resource_size(ledresource[2]));
	GPIO1_DR = ioremap(ledresource[3]->start, resource_size(ledresource[3]));
	GPIO1_GDIR = ioremap(ledresource[4]->start, resource_size(ledresource[4]));

	/* 2、使能GPIO1时钟 */
	val = readl(IMX6U_CCM_CCGR1);
	val &= ~(3 << 26);
	val |= (3 << 26);
	writel(val,IMX6U_CCM_CCGR1);

	/* 3、设置GPIO1_IO03的复用功能，将其复用为
	*    GPIO1_IO03，最后设置IO属性。
	*/
	writel(5, SW_MUX_GPIO1_IO03);
	writel(0x10B0, SW_PAD_GPIO1_IO03);

	/* 4、设置GPIO1_IO03为输出功能 */
	val = readl(GPIO1_GDIR);
	val &= ~(1 << 3);
	val |= (1 << 3);
	writel(val, GPIO1_GDIR);

	/* 注册字符设备驱动 */
	/* 1、创建设备号 */
	if(newchrled.major){
		newchrled.devid = MKDEV(newchrled.major, 0);
		register_chrdev_region(newchrled.devid, PLATFORM_CNT, PLATFORM_NAME);

	}else {
		alloc_chrdev_region(&newchrled.devid, 0, PLATFORM_CNT, PLATFORM_NAME);
		newchrled.major = MAJOR(newchrled.devid);
		newchrled.minor = MINOR(newchrled.devid);
	}
	printk("newchrled major=%d.minor=%d\r\n", newchrled.major, newchrled.minor);

	/* 2、初始化cdev */
	newchrled.cdev.owner = THIS_MODULE;
	cdev_init(&newchrled.cdev, &platform_fops);

	/* 3、添加一个cdev */
	cdev_add(&newchrled.cdev, newchrled.devid, PLATFORM_CNT);

	/* 4、创建类 */
	newchrled.class = class_create(THIS_MODULE, PLATFORM_NAME);
	if(IS_ERR(newchrled.class)){
		return PTR_ERR(newchrled.class);
	}

	/* 5、创建设备 */
	newchrled.device = device_create(newchrled.class, NULL, newchrled.devid, NULL, PLATFORM_NAME);
	if (IS_ERR(newchrled.device)) {
		return PTR_ERR(newchrled.device);
	}

	return 0;
}

static int led_remove(struct platform_device *dev){
	/* 取消映射 */
	iounmap(IMX6U_CCM_CCGR1);
	iounmap(SW_MUX_GPIO1_IO03);
	iounmap(SW_PAD_GPIO1_IO03);
	iounmap(GPIO1_DR);
	iounmap(GPIO1_GDIR);

	cdev_del(&newchrled.cdev);
	unregister_chrdev_region(newchrled.devid, PLATFORM_CNT);

	device_destroy(newchrled.class, newchrled.devid);
	class_destroy(newchrled.class);
	return 0;
}
static struct platform_driver led_driver = {
	.driver = {
		.name = "imx6ull-led1",
	},
	.probe = led_probe,
	.remove = led_remove,
};

static int __init leddrv_init(void){
	return platform_driver_register(&led_driver);
}

static void __exit leddrv_exit(void){
	platform_driver_unregister(&led_driver);
}


module_init(leddrv_init);
module_exit(leddrv_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");