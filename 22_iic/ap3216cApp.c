#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <linux/input.h>

/**
 * 文件名    :ap3216cApp.c
 * 作者      ：azumid
 * 版本      :V1.0
 * 描述      ：ap3216c IIC驱动测试APP
 * 使用     ：./ap3216cApp /dev/ap3216c
 *             
 *             
*/

int main(int argc, char* argv[])
{
    int fd,err;
    char *filename;
    unsigned short data[3];
    unsigned short ir, ps, als;

    if(argc != 2){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}
    
    while(1){
        err = read(fd, data, sizeof(data));
        if(err == 0){
            ir = data[0];
            als = data[1];
            ps = data[2];
            printf("AP3216C ir = %d, als = %d, ps = %d \r\n", ir, als, ps);
        }
        usleep(200000);     /* 200ms */
    }

    close(fd);

    return 0;
}