#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

/**
 * 文件名    :ledApp.c
 * 作者      ：azumid
 * 版本      :V1.0
 * 描述      ：leddevbase驱动测试APP
 * 其他      ：使用方法：./ledtest /dev/leddevbase 0 关闭LED
 *                      ./ledtest /dev/leddevbase 1 打开LED
 *             
*/
#define LEDOFF  0
#define LEDON   1

int main(int argc, char* argv[])
{
    int fd,ret;
    char *filename;
    unsigned char databuf[1];
    int cnt = 0;
    if(argc != 3){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}
    
    databuf[0] = atoi(argv[2]);
    
    /*向/dev/leddevbase文件写入数据*/
    ret = write(fd, databuf, sizeof(databuf));
    if(ret < 0)
    {
        printf("LED Control Failed!\r\n");
        close(fd);
        return -1;
    }
    
    /* 模拟占用25s LED */
    while(1){
        sleep(5);
        cnt++;
        printf("App running times:%d\r\n",cnt);
        if(cnt >= 5) break;
    }

    printf("App running finished!\r\n");

    ret = close(fd);
    if(ret < 0)
    {
        printf("file %s close failed!\r\n",filename);
        return -1;
    }

    return 0;
}