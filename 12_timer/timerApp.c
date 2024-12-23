#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "linux/ioctl.h"

/**
 * 文件名    :ledApp.c
 * 作者      ：azumid
 * 版本      :V1.0
 * 描述      ：leddevbase驱动测试APP
 * 其他      ：使用方法：./ledtest /dev/leddevbase 0 关闭LED
 *                      ./ledtest /dev/leddevbase 1 打开LED
 *             
*/
#define CLOSE_CMD       (_IO(0xEF, 0x1))
#define OPEN_CMD        (_IO(0xEF, 0x2))
#define SETPERIOD_CMD   (_IO(0xEF, 0x3))

int main(int argc, char* argv[])
{
    int fd,ret;
    char *filename;
    unsigned int cmd,arg;
    unsigned char str[100];

    if(argc != 2){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}

    while(1){
        printf("Input CMD:");
        ret = scanf("%d", &cmd);
        if(ret != 1){   /* 参数输入错误 */
            gets(str);  /* 防止卡死     */
        }

        if(cmd == 1){
            cmd = CLOSE_CMD;
        } else if(cmd == 2){
            cmd = OPEN_CMD;
        } else if(cmd == 3){
            cmd = SETPERIOD_CMD;
            printf("Input Timer Period:");
            ret = scanf("%d", &arg);
            if(ret != 1){
                gets(str);
            }
        }
        ioctl(fd, cmd , arg);
    } 
    
    ret = close(fd);
    if(ret < 0)
    {
        printf("file %s close failed!\r\n",filename);
        return -1;
    }

    return 0;
}