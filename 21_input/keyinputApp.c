#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <linux/input.h>

/**
 * 文件名    :keyinputApp.c
 * 作者      ：azumid
 * 版本      :V1.0
 * 描述      ：leddevbase驱动测试APP
 * 使用     ：./keyinputApp /dev/input/event1
 * 注意     ：读取/dev/input/event1，会返回input_event结构体类型的数据！！！            
 *             
*/

/* input_event结构体 */
static struct input_event inputevent;

int main(int argc, char* argv[])
{
    int fd,err;
    char *filename;

    if(argc != 2){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}
    
    while(1){
        err = read(fd, &inputevent, sizeof(inputevent));
        if(err > 0){
            switch(inputevent.type){
                case EV_KEY:
                    if(inputevent.code < BTN_MISC){ /* 这是key */
                        printf("key %d %s!\r\n", inputevent.code, inputevent.value?"press":"release");
                    }else{  /* 这是button */
                        printf("button %d %s!\r\n", inputevent.code, inputevent.value?"press":"release");
                    }
                    //printf("EV_KEY事件!\r\n");
                    break;
                case EV_SYN:
                    //printf("EV_SYN事件!\r\n");
                    break;
                case EV_REL:
                    //printf("EV_REL事件!\r\n");
                    break;
                case EV_ABS:
                    //printf("EV_ABS事件!\r\n");
                    break;
                default:
                    //printf("其他事件!\r\n");
                    break;
            }
        } else printf("数据读取失败!\r\n");
    }

    close(fd);

    return 0;
}