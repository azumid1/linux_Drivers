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

#define GT_CTRL_REG 	        0X8040  /* GT9147控制寄存器         */
#define GT_MODSW_REG 	        0X804D  /* GT9147模式切换寄存器        */
#define GT_9xx_CFGS_REG 	        0X8047  /* GT9147配置起始地址寄存器    */
#define GT_1xx_CFGS_REG 	        0X8050  /* GT1151配置起始地址寄存器    */
#define GT_CHECK_REG 	        0X80FF  /* GT9147校验和寄存器       */
#define GT_PID_REG 		        0X8140  /* GT9147产品ID寄存器       */

#define GT_GSTID_REG 	        0X814E  /* GT9147当前检测到的触摸情况 */
#define GT_TP1_REG 		        0X814F  /* 第一个触摸点数据地址 */
#define GT_TP2_REG 		        0X8157	/* 第二个触摸点数据地址 */
#define GT_TP3_REG 		        0X815F  /* 第三个触摸点数据地址 */
#define GT_TP4_REG 		        0X8167  /* 第四个触摸点数据地址  */
#define GT_TP5_REG 		        0X816F	/* 第五个触摸点数据地址   */
#define MAX_SUPPORT_POINTS      5       /* 最多5点电容触摸 */


struct gt9147_dev {
    int irq_pin, reset_pin;
    int irq_num;
    int irqtype;
    int max_x;      /* 最大横坐标   	*/
    int max_y;      /* 最大纵坐标		*/
    void *private_data;
    struct input_dev *input;
    struct i2c_client *client;

};

struct gt9147_dev gt9147;

const u8 irq_table[] = {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH};  /* 触发方式 */

static int gt9147_ts_reset(struct i2c_client *client, struct gt9147_dev *dev){
    int ret = 0;
    /* 申请复位IO */
    if(gpio_is_valid(dev->reset_pin)){
        /* 申请复位IO，并且默认输出高电平 */
        /**
         * 这里要注意client->dev和dev是不一样的。
         * client->dev存储着设备树中的设备节点的信息
         * dev是我们自己定义的一个结构体，信息是不全的
         * 千万不能用dev代替client->dev！！！
         */
        ret = devm_gpio_request_one(&client->dev,
                        dev->reset_pin, GPIOF_OUT_INIT_HIGH,
                        "gt9147 reset");
        if(ret) return ret;    
    }
    /* 申请中断IO */
    if(gpio_is_valid(dev->irq_pin)){
        /* 申请复位IO，并且默认输出高电平 */
        ret = devm_gpio_request_one(&client->dev,
                    dev->irq_pin, GPIOF_OUT_INIT_HIGH,
                    "gt9147 int");
        
        if(ret) return ret;
    }
    
    /* 初始化GT9147,要严格按照GT9147的时许要求 */
    gpio_set_value(dev->reset_pin, 0);  /* 复位GT9147 */
    msleep(10);
    gpio_set_value(dev->reset_pin, 0);  /* 停止复位GT9147 */
    msleep(10);
    gpio_set_value(dev->irq_pin, 0);    /* 拉低INT引脚 */
    gpio_direction_input(dev->irq_pin); /* INT引脚设置为输入 */

    return 0;
}



static int gt9147_read_regs(struct gt9147_dev *dev, u16 reg, u8 *buf, int len){
    int ret;
    u8 data[2];
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client*)dev->client;

    data[0] = reg >> 8;
    data[1] = reg & 0xFF;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = data;
    msg[0].len = 2;

    /* msg[1]读取数据 */
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = len;
    ret = i2c_transfer(client->adapter, msg, 2);
    if(ret == 2){
        ret = 0;
    } else {
        ret = -EREMOTEIO;
    }

    return ret;
}

static int gt9147_write_regs(struct gt9147_dev *dev, u16 reg, u8 *buf, u8 len){
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client*)dev->client;

    b[0] = reg >> 8;    /* 寄存器首地址高8位 */
    b[1] = reg & 0xFF;  /* 寄存器的地址低8位 */
    memcpy(&b[2], buf, len);    /* 将要写入的数据拷贝到数组b中 */
    
    msg.addr = client->addr;    /* gt9147的地址 */
    msg.flags = 0;              /* 标记为写数据 */
    msg.buf = b;                /* 要写的数据的缓冲区 */
    msg.len = len + 2;          /* 要写的数据长度 */

    return i2c_transfer(client->adapter, &msg, 1);

}

static irqreturn_t gt9147_irq_handler(int irq, void *dev_id){
    int touch_num = 0;
    int input_x, input_y;
    int id = 0;
    int ret = 0;
    u8 data;
    u8 touch_data[5];
    struct gt9147_dev *dev = dev_id;

    ret = gt9147_read_regs(dev, GT_GSTID_REG, &data, 1);
    if(data == 0x00){   /* 没有触摸数据，直接返回 */
        goto fail;
    } else {
        touch_num = data & 0xF;  /* 统计触摸点数据 */
    }

    /* 由于GT9147没有硬件检测每个触摸点按下和抬起，因此每个触摸点的抬起和按
     * 下不好处理，尝试过一些方法，但是效果都不好，因此这里暂时使用单点触摸 
     */
    if(touch_num){   /* 单点触摸按下 */
        /* 这里我觉得寄存器读错了，应该读的是0x8157 */
        //gt9147_read_regs(dev, GT_TP1_REG, touch_data, 5);
        gt9147_read_regs(dev, 0x8157, touch_data, 5);
        id = touch_data[0] & 0xF;
        if(id == 0){
            input_x = (touch_data[2] << 8) | touch_data[1];
            input_y = (touch_data[4] << 8) | touch_data[3];

            /* 下面这四步对应typeB触摸点的数据上报时序 */
            input_mt_slot(dev->input, id);
            input_mt_report_slot_state(dev->input, MT_TOOL_FINGER, true);
            input_report_abs(dev->input, ABS_MT_POSITION_X, input_x);
            input_report_abs(dev->input, ABS_MT_POSITION_Y, input_y);
        } else if(touch_num == 0){           /* 单点触摸释放 */
            input_mt_slot(dev->input, id);
            input_mt_report_slot_state(dev->input, MT_TOOL_FINGER, false);
        }

        input_mt_report_pointer_emulation(dev->input, true);
        input_sync(dev->input);

        data = 0x00;
        gt9147_write_regs(dev, GT_GSTID_REG, &data, 1);
    }
fail:
	return IRQ_HANDLED;
}

static int gt9147_ts_irq(struct i2c_client *client, struct gt9147_dev *dev){
    int ret = 0;
    /* 申请中断，client->irq就是IO中断 */
    ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
                            gt9147_irq_handler, irq_table[dev->irqtype] | IRQF_ONESHOT,
                            client->name, &gt9147);
    if(ret){
        dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
        return ret;
    }

    return 0;
}


/* 读取固件信息 */
static int gt9147_read_firmware(struct i2c_client *client, struct gt9147_dev *dev){
    int ret = 0, version = 0;
    u16 id = 0;
    u8 data[7] = {0};
    char id_str[5];
    ret = gt9147_read_regs(dev, GT_PID_REG, data, 6);
    if(ret){
        dev_err(&client->dev, "Unable to read PID.\r\n");
        return ret;
    }
    memcpy(id_str, data, 4); /* 这前四个存储着Product ID */
    id_str[4] = 0;
    /*  
    如果kstrtou16返回非零值（即转换失败），则将id设置为0x1001。
    这意味着如果id_str不是一个有效的十进制数，id将被设置为0x1001。 
    */
    /*
    还要注意一点，kstrtou16是将整个数组（数组的内容组成字符串）转换成u16  
    */
    if(kstrtou16(id_str, 10, &id))
        id = 0x1001;

    version = get_unaligned_le16(&data[4]);
    dev_info(&client->dev, "ID %d, version:%04x\n", id, version);
    switch (id) {    /* 由于不同的芯片配置寄存器地址不一样需要判断一下  */
    
    case 1151:
    case 1158:
    case 5663:
    case 5688:    /* 读取固件里面的配置信息  */
    /* 其实前面这几个情况是为了兼容gt1xxx芯片的 */
        ret = gt9147_read_regs(dev, GT_1xx_CFGS_REG, data, 7);
        /* 执行完上面的语句后，data存储的内容是从0x8047开始的，其中data[1]~data[4]存储着 */  
		break;
    /* 默认会执行gt9147的寄存器读取 */    
    default:
        ret = gt9147_read_regs(dev, GT_9xx_CFGS_REG, data, 7);
		break;
    }
    if(ret){
        dev_err(&client->dev, "Unable to read Firnware.\n");
        return ret;
    }
    /* 这样处理的原因是寄存器中先存储的低位数据 */
    dev->max_x = (data[2] << 8) + data[1];
    dev->max_y = (data[4] << 8) + data[3];
    /**
    INT 触发方式
    00：上升沿触发
    01：下降沿触发
    02：低电平查询
    03：高电平查询
    这里设置的是高电平查询
     */
    dev->irqtype = data[6] & 0x03;
    printk("X_MAX: %d, Y_MAX: %d, TRIGGER:0x%02x", dev->max_x, dev->max_y, dev->irqtype);

    return 0;
}

static int gt9147_probe(struct i2c_client *client, const struct gt9147_dev *dev){
    u8 ret = 0;
    u8 data;
    gt9147.client = client;
    /* 1.获取设备书中的中断和复位引脚 */
    gt9147.irq_pin = of_get_named_gpio(client->dev.of_node, "interrupt-gpios", 0);
    gt9147.reset_pin = of_get_named_gpio(client->dev.of_node, "reset-gpios", 0);

    /* 2.复位GT9147 */
    ret = gt9147_ts_reset(client, &gt9147);
    if(ret < 0) goto fail;

    /* 3.初始化GT9147 */
    data = 0x02;
    gt9147_write_regs(&gt9147, GT_CTRL_REG, &data, 1);  /* 软复位 */
    msleep(100);
    data = 0x0;
    gt9147_write_regs(&gt9147, GT_CTRL_REG, &data, 1);  /* 复位结束 */
    msleep(100);

    /* 4.初始化GT9147,读取固件 */
    ret = gt9147_read_firmware(client, &gt9147);
    if(ret != 0){
        printk("Fail !!! check !!\r\n");
        goto fail;
    }

    /* 5.注册input设备 */
    gt9147.input = devm_input_allocate_device(&client->dev);
    if(!gt9147.input){
        ret = -ENOMEM;
        goto fail;
    }
    gt9147.input->name = client->name;
    gt9147.input->id.bustype = BUS_I2C;
    gt9147.input->dev.parent = &client->dev;

    __set_bit(EV_KEY, gt9147.input->evbit);
    __set_bit(EV_ABS, gt9147.input->evbit);

    /* 它告诉内核，gt9147 设备支持“触摸按钮”事件。
    当用户触摸屏幕时，该设备可以生成 BTN_TOUCH 事件，
    内核可以根据这个事件进行相应的处理（如触发触摸屏操作）。 */
    __set_bit(BTN_TOUCH, gt9147.input->keybit);     /* BTN_TOUCH表示触摸屏的触摸事件 */

    input_set_abs_params(gt9147.input, ABS_X, 0, gt9147.max_x, 0, 0);
    input_set_abs_params(gt9147.input, ABS_Y, 0, gt9147.max_y, 0, 0);
    input_set_abs_params(gt9147.input, ABS_MT_POSITION_X, 0, gt9147.max_x, 0, 0);
    input_set_abs_params(gt9147.input, ABS_MT_POSITION_Y,0, gt9147.max_y, 0, 0);
    ret = input_mt_init_slots(gt9147.input, MAX_SUPPORT_POINTS, 0);
    if(ret){
        goto fail;
    }

    ret = input_register_device(gt9147.input);  /* 当输入设备被注册时，这个函数会在内核日志中打印设备信息，包括设备的路径。 */
    if(ret){
        goto fail;
    }

    /* 6.最后初始化中断 */
    ret = gt9147_ts_irq(client, &gt9147);

    return 0;
fail:
    return ret;

}

static int gt9147_remove(struct i2c_client *client){
    input_unregister_device(gt9147.input);
    return 0;
}

/* 传统的设备树匹配列表 */
const struct i2c_device_id gt9147_id_table[] = {
    {"goodix,gt9147", 0},
    {}
};

/* 设备树匹配表 */
const struct of_device_id gt9147_mathc_table[] = {
    {.compatible = "goodix,gt9147"},
    {}
};


/* i2c驱动结构体 */
struct i2c_driver gt9147_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "gt9147",
        .of_match_table = gt9147_mathc_table,
    },
    .id_table = gt9147_id_table,
    .probe = gt9147_probe,
    .remove = gt9147_remove,
};
module_i2c_driver(gt9147_i2c_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("azumid");
