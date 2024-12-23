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

#if 0
alphaled {
		#address-cells = <1>;
		#size-cells = <1>;
		compatible = "atkalpha-led";
		status = "okay";
		reg = <	0x020C406C	0x04		/* CCM_CCGR1_BASE 			*/
				0x020E0068	0x04		/* SW_MUX_GPIO1_IO03_BASE 	*/
				0x020E02F4	0x04		/* SW_PAD_GPIO1_IO03_BASE 	*/
				0x0209C000	0x04		/* GPIO1_DR_BASE			*/
				0x0209C004	0x04>;		/* GPIO1_GDIR_BASE			*/
	};
#endif

/*映射后的寄存器虚拟地址指针*/
static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

#define DTSLEDNAME      "dtsled"
#define DTSLEDCNT        1
#define LEDOFF           0
#define LEDON            1

/* dtsled结构体 */
struct dtsled_dev{
    struct cdev cdev;           /* 字符设备     */
    struct class *class;        /* 类           */
    struct device *device;      /* 设备         */
    dev_t devid;                /* 设备号       */
    int major;                  /* 主设备号     */
    int minor;                  /* 次设备号     */
    struct device_node *nd;     /* 节点信息     */
};

struct dtsled_dev dtsled;

void led_switch(u8 stat){
    u32 val;
    if(stat == LEDON){
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);
        writel(val, GPIO1_DR);
    } else if(stat == LEDOFF){
        val = readl(GPIO1_DR);
        val |= (1 << 3);
        writel(val, GPIO1_DR);
    }
}
static int dtsled_open(struct inode *inode, struct file *filp){
    filp->private_data = &dtsled;       /*  这是设置了文件私有数据  */
    return 0;
}

static ssize_t dtsled_write(struct file *filp, const char __user *buf,
                            size_t cnt, loff_t *offt)
{
    struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;
    int ret;
    unsigned char databuf[1];
    unsigned char ledstat;

    ret = copy_from_user(databuf, buf, cnt);
    if(ret < 0){
        printk("kernle write failed\r\n");
        return -EFAULT;
    }
    ledstat = databuf[0];
    led_switch(ledstat);
    return 0;

}
static int dtsled_release(struct inode *inode, struct file *filp){
    struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;    /*  这是提取了私有数据  */
    return 0;
}

static struct file_operations dtsled_fops = {
    .owner = THIS_MODULE,
    .open = dtsled_open,
    .write = dtsled_write,
    .release = dtsled_release,
};

static int __init dtsled_init(void)
{
    int ret;
    dtsled.nd = NULL;
    const char *str;
    u32 *arr;
    u8 elemsize;
    u8 i;
    u32 val;

    /* 注册新字符设备号 */
    dtsled.major = 0;       /* 设备号由内核分配 */
    if(dtsled.major){       /* 设备号已经给定   */
        dtsled.devid = MKDEV(dtsled.major, 0);
        ret = register_chrdev_region(dtsled.devid, DTSLEDCNT, DTSLEDNAME);
    } else {                /* 没有给定设备号   */
        ret = alloc_chrdev_region(&dtsled.devid, 0, DTSLEDCNT, DTSLEDNAME);
        dtsled.major = MAJOR(dtsled.devid);
        dtsled.minor = MINOR(dtsled.devid);
    }

    if(ret < 0){
        goto fail_devid;
    }

    /* 添加字符设备 */
    dtsled.cdev.owner = THIS_MODULE;
    cdev_init(&dtsled.cdev, &dtsled_fops);
    ret = cdev_add(&dtsled.cdev, dtsled.devid, DTSLEDCNT);
    if(ret < 0){
        goto fail_cdevadd;
    } 

    /* 自动创建设备节点 */
    dtsled.class = class_create(THIS_MODULE, DTSLEDNAME);
    if(IS_ERR(dtsled.class)){
        ret = PTR_ERR(dtsled.class);
        goto fail_classcrt;
    }

    dtsled.device = device_create(dtsled.class, NULL, dtsled.devid, NULL, DTSLEDNAME);
    if(IS_ERR(dtsled.device)){
        ret = PTR_ERR(dtsled.device);
        goto fail_devicecrt;
    }

    /* 先找到需要的节点*/
    dtsled.nd = of_find_node_by_path("/alphaled");
    if(dtsled.nd == NULL){
        ret = -EINVAL;
        goto fail_findnd;
    }

    /*  打印status属性的内容    */
    ret = of_property_read_string(dtsled.nd, "status", &str);
    if(ret < 0){
        goto fail_rdstr;
    } else{
        printk("status = %s\r\n",str);
    }

    // /*  获取reg属性的内容   */
    // elemsize = of_property_count_elems_of_size(dtsled.nd, "reg", sizeof(u32));        /* 查找元素的数量 */
    // if(ret < 0){
    //     ret = -EINVAL;
    //     goto fail_fdelem;
    // } else{
    //     printk("reg elems = %d\r\n", elemsize);
    // }   
    // arr = (u32*)kmalloc(sizeof(u32)*elemsize, GFP_KERNEL);          /* 先申请内存 */
    // if(arr == NULL){
    //     printk("arr err!\r\n");
    // }
    // ret = of_property_read_u32_array(dtsled.nd, "reg", arr, elemsize);
    // if(ret < 0)
    // {
    //     goto fail_rdarray;
    // } else{
    //     printk("reg data:\r\n");
    //     for(i=0;i<elemsize;i++){
    //         printk("%#X ",*(arr + i));
    //     }
    //     printk("\r\n");
    // }

    // /* 1.寄存器地址映射 */
    // IMX6U_CCM_CCGR1 = ioremap(*(arr), 4);
    // SW_MUX_GPIO1_IO03 = ioremap(*(arr+2), 4);
    // SW_PAD_GPIO1_IO03 = ioremap(*(arr+4), 4);
    // GPIO1_DR = ioremap(*(arr+6), 4);
    // GPIO1_GDIR = ioremap(*(arr+8), 4);

    /* 1.寄存器地址映射 */
    IMX6U_CCM_CCGR1 = of_iomap(dtsled.nd, 0);
    SW_MUX_GPIO1_IO03 = of_iomap(dtsled.nd, 1);
    SW_PAD_GPIO1_IO03 = of_iomap(dtsled.nd, 2);
    GPIO1_DR = of_iomap(dtsled.nd, 3);
    GPIO1_GDIR = of_iomap(dtsled.nd, 4);

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
    val &= ~(1 << 3);
    val |= (1 << 3);
    writel(val, GPIO1_GDIR); 

    //kfree(arr);

    return 0;

fail_rdarray:
    //kfree(arr);
fail_fdelem:
fail_rdstr:
fail_findnd:
    device_destroy(dtsled.class, dtsled.devid);
fail_devicecrt:
    class_destroy(dtsled.class);
fail_classcrt:
    cdev_del(&dtsled.cdev);
fail_cdevadd:
    unregister_chrdev_region(dtsled.devid, DTSLEDCNT);
fail_devid:
    return ret;

  
}

static void __exit dtsled_exit(void)
{
    /* 删除字符设备 */
    cdev_del(&dtsled.cdev);

    /* 删除设备号   */
    unregister_chrdev_region(dtsled.devid, DTSLEDCNT);

    /* 注销设备节点 */
    device_destroy(dtsled.class, dtsled.devid);

    /* 删除类       */
    class_destroy(dtsled.class);
}

/* 注册和卸载驱动 */
module_init(dtsled_init);
module_exit(dtsled_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");