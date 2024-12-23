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

/* 寄存器物理地址 */
#define CCM_CCGR1_BASE			(0x020C406C)
#define SW_MUX_GOIO1_IO03_BASE	(0x020E0068)
#define SW_PAD_GPIO1_IO01_BASE	(0x020E02F4)
#define GPIO_DR_BASE			(0x0209C000)
#define GPIO_GDIR_BASE			(0x0209C004)
#define REGISTER_LENGTH			4

/* 这是向platform中注册一个设备 */
#if 0
struct platform_device {
	const char	*name;
	int		id;
	bool		id_auto;
	struct device	dev;
	u32		num_resources;
	struct resource	*resource;

	const struct platform_device_id	*id_entry;
	char *driver_override; /* Driver name to force a match */

	/* MFD cell pointer */
	struct mfd_cell *mfd_cell;

	/* arch specific additions */
	struct pdev_archdata	archdata;
};
#endif

void leddevice_release(struct device *dev){
	printk("leddeice release!\r\n");
}

static struct resource led_resource[] = {
	[0] = {
		.start = CCM_CCGR1_BASE,
		.end = CCM_CCGR1_BASE + REGISTER_LENGTH - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SW_MUX_GOIO1_IO03_BASE,
		.end = SW_MUX_GOIO1_IO03_BASE + REGISTER_LENGTH - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = SW_PAD_GPIO1_IO01_BASE,
		.end = SW_PAD_GPIO1_IO01_BASE + REGISTER_LENGTH -1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.start = GPIO_DR_BASE,
		.end = GPIO_DR_BASE + REGISTER_LENGTH - 1,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.start = GPIO_GDIR_BASE,
		.end = GPIO_GDIR_BASE + REGISTER_LENGTH - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device leddevice = {
	.name = "imx6ul-led",
	.id = -1,	/* -1代表系统自动分配设备id */
	.dev = {
		.release = leddevice_release,
	},
	.num_resources = ARRAY_SIZE(led_resource),
	.resource = led_resource,
};

static int __init leddev_init(void){
	return platform_device_register(&leddevice);
}

static void __exit leddev_exit(void){
	platform_device_unregister(&leddevice);
}


module_init(leddev_init);
module_exit(leddev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");

