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

/**
 * @description    :LED打开/关闭
 * @param - sta    :LEDON(0)打开LED，LEDOFF(1)关闭LED
 * @return         :无
*/
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

/**
 * @description     :打开设备
 * @param - inode   :传递给驱动的inode
 * @param - filp    :设备文件，file结构体有个叫做private_data的
 *                   成员变量，一般在open的时候将private_data指向
 *                   设备结构体。
 * @return          :0 成功 ；其他  失败
*/
static int led_open(struct inode *inode, struct file *filp)
{
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
static ssize_t led_write(struct file *filp, const char __user *buf,
                        size_t cnt, loff_t *offt)
{
    int retval;
    unsigned char databuf[1];
    unsigned char ledstat;
    retval = copy_from_user(databuf, buf, cnt);
    if(retval < 0)
    {
        printk("kernel write failed!\r\n");
        return -EFAULT;
    }
    
    ledstat = databuf[0];

    if(ledstat == LEDON)
    {
        led_switch(LEDON);
    }
    else if(ledstat == LEDOFF)
    {
        led_switch(LEDOFF);
    }

    return 0;
}
static int led_release(struct inode *inode, struct file *filp)
{
    // u32 val;
    // /*5.LED开关设置*/
    // val = readl(GPIO1_DR);
    // val &= ~(1 << 3);          /*bit3为1则输出高电平，为0则输出低电平*/
    // writel(val, GPIO1_DR);
    return 0;
}
/*字符设备操作集*/
static struct file_operations led_fops ={
    .owner = THIS_MODULE,
    .open = led_open,
    .write = led_write,
    .release = led_release,
};

static int __init led_init(void)
{
    u32 val = 0;
    int ret = 0;
    /* 初始化LED*/
    /* 1、寄存器地址映射*/
    IMX6U_CCM_CCGR1 = ioremap(CCM_CCGR1_BASE, 4);
    SW_MUX_GPIO1_IO03 = ioremap(SW_MUX_GPIO1_IO03_BASE, 4);
    SW_PAD_GPIO1_IO03 = ioremap(SW_PAD_GPIO1_IO03_BASE, 4);
    GPIO1_DR = ioremap(GPIO1_DR_BASE, 4);
    GPIO1_GDIR = ioremap(GPIO1_GDIR_BASE, 4);
    
    /*2、使能GPIO1时钟*/
    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);
    val |= (3 << 26);
    writel(val, IMX6U_CCM_CCGR1);

    /**
     * 3.设置GPIO1_IO03的复用功能 
     * 将其复用为GPIO1_IO03，最后设置IO属性
    */
    writel(5, SW_MUX_GPIO1_IO03);

    /*设置IO属性*/
    writel(0x10B0, SW_PAD_GPIO1_IO03);

    /* 4.设置GPIO1_IO03为输出功能*/
    val = readl(GPIO1_GDIR);
    //val &= ~(1 << 3);
    val |= (1 << 3);
    writel(val, GPIO1_GDIR);

    /*5.LED开关设置*/
    val = readl(GPIO1_DR);
    val |= (1 << 3);          /*bit3为1则输出高电平，为0则输出低电平*/
    writel(val, GPIO1_DR);

    
    /*6.注册字符设备*/
    ret = register_chrdev(LEDDEVBASE_MAJOR, LEDDEVBASE_NAME, &led_fops);
    if(ret < 0)
    {
        printk("register failed!\r\n");
        return -EIO;
    }

    return 0;
}

static void __exit led_exit(void)
{

    /*取消映射*/
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    /*注销字符设备*/
    unregister_chrdev(LEDDEVBASE_MAJOR, LEDDEVBASE_NAME);
    
}

/*注册驱动的加载和卸载*/
module_init(led_init);
module_exit(led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");