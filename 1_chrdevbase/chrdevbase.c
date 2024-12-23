#include<linux/module.h>
#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/fs.h>
#include<linux/types.h>
#include<linux/delay.h>
#include<linux/ide.h>

/**
 * 文件名: chrdevbase.c
 * 作者：  azumid
 * 版本：  1.0
 * 描述：  chrdevbase 驱动文件
 * 
 */

#define CHARDEVBASE_MAJOR   200
#define CHARDEVBASE_NAME    "chrdevbase"

static char readbuf[100];
static char writebuf[100];
static char kerneldata[] = {"kernel data!"};


static int chrdevbase_open(struct inode* inode, struct file* filp)
{
    //printk("chrdevbase_open\r\n");
    return 0;
}

// static int chrdevbase_release(struct inode* inode, struct file* filp)
// {
//     printk("chrdevbase_release\r\n");
//     return 0;
// }

/**
 * @description:     从设备读取数据
 * @param - filp :   要打开的设备文件（文件描述符）
 * @param - buf  :   返回给用户空间的数据缓冲区
 * @param - cnt  :   要读取的数据长度
 * @param - offt :   文件指针的偏移量
 * @return       :   读取的字节数，如果为负值，表示读取失败
*/

static ssize_t chrdevbase_read(struct file* filp, const char __user *buf,size_t count, loff_t *ppos)
{
    int ret = 0;

    /* 向用户空间发送数据*/
    memcpy(readbuf, kerneldata, sizeof(kerneldata));
    ret = copy_to_user(buf, readbuf, count);
    if(ret == 0) 
    {
        printk("kernel senddata okay!\r\n");
    }
    else 
    {
        printk("kernel senddata failed\r\n");
    }
    
    return 0;

}

/**
 * @description:     从设备写数据
 * @param - filp :   要打开的设备文件（文件描述符）
 * @param - buf  :   返回给用户空间的数据缓冲区
 * @param - cnt  :   要写的数据长度
 * @param - offt :   文件指针的偏移量
 * @return       :   写入的字节数，如果为负值，表示读取失败
*/
static ssize_t chrdevbase_write(struct file* filp, const char __user *buf,size_t count, loff_t *ppos)
{
    int ret = 0;
    
    /*接收用户空间传递给内核的数据并且打印出来*/
    ret = copy_from_user(writebuf, buf, count);
    if(ret == 0) 
    {
        printk("kernel recevdata:%s\r\n",writebuf);
        
    }
    else 
    {
        printk("kernel recevdata failed\r\n");
    }
    //printk("chrdevbase_write\r\n");
    return 0;

}

static int chrdevbase_release(struct inode *inode, struct file *filp)
{
	//printk("chrdevbase release！\r\n");
	return 0;
}

static struct file_operations chrdevbase_fops={
    .owner = THIS_MODULE,
    .open = chrdevbase_open,
    .read = chrdevbase_read,
    .write = chrdevbase_write,
    .release = chrdevbase_release,
    
};

static int __init chrdevbase_init(void)
{
    /*注册字符设备*/
    int ret = 0; 
    ret = register_chrdev(CHARDEVBASE_MAJOR,CHARDEVBASE_NAME,&chrdevbase_fops);
    if(ret < 0)
    {
        printk("chrdevbase init failed\r\n");
    }
    else
    {
        printk("chrdevbase_init()\r\n");
    } 
    return 0;
}

static void __exit chrdevbase_exit(void)
{
    /*注销字符设备*/
    unregister_chrdev(CHARDEVBASE_MAJOR,CHARDEVBASE_NAME);
    printk("exit succeed!\r\n");
}

/*
 模块入口与出口
*/
module_init(chrdevbase_init);
module_exit(chrdevbase_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");