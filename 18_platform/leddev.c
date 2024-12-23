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
#include<linux/fcntl.h>
#include<linux/platform_device.h>

/* 寄存器物理地址 */
#define CCM_CCGR1_BASE				(0X020C406C)	
#define SW_MUX_GPIO1_IO03_BASE		(0X020E0068)
#define SW_PAD_GPIO1_IO03_BASE		(0X020E02F4)
#define GPIO1_DR_BASE				(0X0209C000)
#define GPIO1_GDIR_BASE				(0X0209C004)
#define REGISTER_LENGHT             4

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
    printk("leddevice release\r\n");
}

#if 0
struct resource{
    resource_size_t start;
    resource_size_t end;
    const char *name;
    unsigned long flags;
    struct resource *parent, *sibling, *child;
};
#endif



static struct resource led_resources[] = {
    [0] = {
        .start = CCM_CCGR1_BASE,
        .end = CCM_CCGR1_BASE + REGISTER_LENGHT - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = SW_MUX_GPIO1_IO03_BASE,
        .end = SW_MUX_GPIO1_IO03_BASE + REGISTER_LENGHT - 1,
        .flags = IORESOURCE_MEM,
    },
    [2] = {
        .start = SW_PAD_GPIO1_IO03_BASE,
        .end = SW_PAD_GPIO1_IO03_BASE + REGISTER_LENGHT - 1,
        .flags = IORESOURCE_MEM,
    },
    [3] = {
        .start = GPIO1_DR_BASE,
        .end = GPIO1_DR_BASE + REGISTER_LENGHT - 1,
        .flags = IORESOURCE_MEM,
    },
    [4] = {
        .start = GPIO1_GDIR_BASE,
        .end = GPIO1_GDIR_BASE + REGISTER_LENGHT - 1,
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device leddevice = {
    .name = "imx6ull-led",
    .id = -1,
    .dev = {
        .release = leddevice_release,
    },
    .num_resources = ARRAY_SIZE(led_resources),
    .resource = led_resources,

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