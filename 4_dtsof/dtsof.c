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
backlight {
		compatible = "pwm-backlight";
		pwms = <&pwm1 0 5000000>;
		brightness-levels = <0 4 8 16 32 64 128 255>;
		default-brightness-level = <6>;
		status = "okay";
	};
#endif
// /* 定义节点路径*/
// #define PATH        "/backlight"

/**
 * 模块入口
*/
static int __init dtsof_init(void)
{
    int ret = 0;
    struct device_node *bl_nd = NULL;
    struct property *compro = NULL;
    const char *str;
    u32 num = 0;
    u32 elemsize;
    u32 *arr;
    u8 i;
    /* 1.找到backlight节点， 路径是 /backlight */
    bl_nd = of_find_node_by_path("/backlight");
    if(bl_nd == NULL)
    {
        ret = -EINVAL;
        goto fail_findnd;
    }

    /* 2.找到节点后获取属性 */
    compro = of_find_property(bl_nd, "compatible", NULL);
    if(compro == NULL)
    {
        ret = -EINVAL;
        goto fail_findpro;
    } else{
        printk("compatible = %s\r\n",(char*)compro->value);
        printk("name = %s\r\n",compro->name);
    }
    ret = of_property_read_string(bl_nd, "status", &str);
    if(ret < 0)
    {
        goto fail_rs;
    } else{
        printk("staus = %s\r\n", str);
    }

    /* 获取数字属性 */
    ret = of_property_read_u32(bl_nd, "default-brightness-level", &num);
    if(ret < 0){
        goto fail_rdu32;
    } else{
        printk("default-brightness-level = %d\r\n", num);
    }

    /* 获取数组类型的属性 */
    elemsize = of_property_count_elems_of_size(bl_nd, "brightness-levels", sizeof(u32));        /* 查找元素的数量 */
    if(ret < 0){
        ret = -EINVAL;
        goto fail_fdelem;
    } else{
        printk("brightness-level elems = %d\r\n", elemsize);
    }

    arr = (u32*)kmalloc(sizeof(u32)*elemsize, GFP_KERNEL);          /* 先申请内存 */
    if(arr == NULL){
        printk("arr err!\r\n");
    }

    ret = of_property_read_u32_array(bl_nd, "brightness-levels", arr, elemsize);
    if(ret < 0)
    {
        goto fail_rdarray;
    } else{
        for(i=0;i<elemsize;i++){
            printk("default-brightness-level[%d] = %d\r\n",i,*(arr + i));
        }
    }

    kfree(arr);             /* 释放内存 */

    return ret;
fail_rdarray:
    kfree(arr);             /* 释放内存 */
fail_fdelem:
fail_rdu32:
fail_rs:
fail_findpro:
fail_findnd:
    return ret;
}

/**
 * 模块出口
*/
static void __exit dtsof_exit(void)
{
    
}

/* 模块入口和出口 */
module_init(dtsof_init);
module_exit(dtsof_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");