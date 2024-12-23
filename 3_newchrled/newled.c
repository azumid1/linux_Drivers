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

#define NEWCHRLED_NAME  "newchrled"
#define NEWCHRLED_CNT    1
/**
 * 文件名: leddevbase.c
 * 作者：  azumid
 * 版本：  1.0
 * 描述：  chrdevbase 驱动文件
 * 
 */

#define LEDDEVBASE_MAJOR      201
#define LEDDEVBASE_NAME       "leddevbase"
#define LEDOFF                0
#define LEDON                 1

/*寄存器物理地址*/
#define CCM_CCGR1_BASE           (0x020C406C)
#define SW_MUX_GPIO1_IO03_BASE   (0x020E0068)
#define SW_PAD_GPIO1_IO03_BASE   (0x020E02F4)
#define GPIO1_DR_BASE            (0x0209C000)
#define GPIO1_GDIR_BASE          (0x0209C004)

/*映射后的寄存器虚拟地址指针*/
static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

struct newchrled_dev{
    struct cdev cdev;       /* 字符设备 */
    struct class *class;     /* 类 */
    struct device *device;  /* 设备 */
    dev_t devid;            /* 设备号 */
    int major;              /* 主设备号 */
    int minor;              /* 次设备号 */

};

struct newchrled_dev newchrled;

void led_switch(u8 sta)
{
    u32 val = 0;
    if(sta == LEDON)
    {
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);
        writel(val, GPIO1_DR);
    }
    else if(sta == LEDOFF)
    {
        val = readl(GPIO1_DR);
        val |= (1 << 3);
        writel(val, GPIO1_DR);
    }
}

static int newchrled_open(struct inode* inode, struct file *filp)
{
    filp->private_data = &newchrled;
    return 0;
}

static int newchrled_release(struct inode *inode, struct file *filp)
{
    struct newchrled_dev *dev = (struct newchrled_dev*)filp->private_data;
    return 0;
}

/**
 * @description     :向设备写数据
 * @param - filp    :设备文件，表示打开的文件描述符
 * @param - buf     :要写给设备写入的数据
 * @param - cnt     :要写入的数据长度
 * @param - offt    :相对于文件首地址的偏移
 * @return          :写入的字节数，如果为负值，表示写入失败
*/
static ssize_t newchrled_write(struct file *filp, const char __user *buf,
                                size_t cnt, loff_t *offt)
{
    int retval;
    unsigned char databuf[1];
    unsigned char ledstat;
    retval = copy_from_user(databuf, buf, cnt);
    if(retval < 0)
    {
        printk("kernel write failed\r\n");
        return -EFAULT;
    }

    ledstat = databuf[0];
    led_switch(ledstat);

    return 0;
}

// static ssize_t newchrled_read(struct file *filp, const char __user *buf,
//                              size_t cnt, loff_t *offt)
// {
//     return 0;
// }

static struct file_operations newchrled_fops ={
    .owner = THIS_MODULE,
    .open = newchrled_open,
    .write = newchrled_write,
    // .read = newchrled_read,
    .release = newchrled_release,
};

static int __init newled_init(void)
{
    u32 val = 0;
    int ret = 0;
    printk("newled_init!\r\n");
    /* 初始化LED */
    /* 1.寄存器地址映射 */
    IMX6U_CCM_CCGR1 = ioremap(CCM_CCGR1_BASE, 4);
    SW_MUX_GPIO1_IO03 = ioremap(SW_MUX_GPIO1_IO03_BASE, 4);
    SW_PAD_GPIO1_IO03 = ioremap(SW_PAD_GPIO1_IO03_BASE, 4);
    GPIO1_DR = ioremap(GPIO1_DR_BASE, 4);
    GPIO1_GDIR = ioremap(GPIO1_GDIR_BASE, 4);

    /* 2.使能GPIO1时钟*/
    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);
    val |= (3 << 26);
    writel(val, IMX6U_CCM_CCGR1);

    /* 3.设置GPIO1_IO03的复用功能
        将其复用为GPIO1_IO03,并设置IO属性
    */
   writel(5, SW_MUX_GPIO1_IO03);

   /* 设置IO属性*/
   writel(0x10B0, SW_PAD_GPIO1_IO03);

   /* 4.设置GPIO1_IO03为输出功能 */
   val = readl(GPIO1_GDIR);
   val |= (1 << 3);
   writel(val, GPIO1_GDIR);

    newchrled.major = 0;    /* 设置为0，表示系统自动分配设备号 */

   /* 注册新字符设备的设备号*/
   if(newchrled.major)      /* 给定了主设备号 */
   {
        newchrled.devid = MKDEV(newchrled.major, 0);
        ret = register_chrdev_region(newchrled.devid, NEWCHRLED_CNT, NEWCHRLED_NAME);

   }
   else                     /* 没有给定主设备号 */
   {
        ret = alloc_chrdev_region(&newchrled.devid, 0, NEWCHRLED_CNT, NEWCHRLED_NAME);
        newchrled.major = MAJOR(newchrled.devid);
        newchrled.minor = MINOR(newchrled.devid);
   }

   if(ret < 0)
   {
        printk("newchrled chrdev_region err!\r\n");
        goto fail_devid;
   }

   printk("newchrled major=%d,minor=%d\n",newchrled.major, newchrled.minor);
   
    /* 注册新字符设备 */
    newchrled.cdev.owner = THIS_MODULE;
    cdev_init(&newchrled.cdev, &newchrled_fops);        /* 初始化 cdev 结构体变量 */
    ret = cdev_add(&newchrled.cdev, newchrled.devid, NEWCHRLED_CNT);
    if(ret < 0)
    {
        goto fail_cdevadd;
    }
    /* 自动创建字符设备节点*/
    newchrled.class = class_create(THIS_MODULE, NEWCHRLED_NAME);
    if(IS_ERR(newchrled.class))
    {
        //return PTR_ERR(newchrled.class);
        goto fail_class;
    }

    newchrled.device = device_create(newchrled.class, NULL, newchrled.devid, NULL, NEWCHRLED_NAME);
    if(IS_ERR(newchrled.class))
    {
        //return PTR_ERR(newchrled.class);
        goto fail_device;
    }

fail_device:
    class_destroy(newchrled.class);

fail_class:
    cdev_del(&newchrled.cdev);
fail_cdevadd:
    unregister_chrdev_region(newchrled.devid, NEWCHRLED_CNT);

fail_devid:
    return -1;

   return 0;
}

static void __exit newled_exit(void)
{
    /* 取消地址映射 */
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    /* 删除字符设备 */
    cdev_del(&newchrled.cdev);

    /* 注销新字符设备的设备号*/
    unregister_chrdev_region(newchrled.devid, NEWCHRLED_CNT);

    /*注销设备节点 */
    device_destroy(newchrled.class, newchrled.devid);

    /* 摧毁类 */
    class_destroy(newchrled.class);

}

module_init(newled_init);
module_exit(newled_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");