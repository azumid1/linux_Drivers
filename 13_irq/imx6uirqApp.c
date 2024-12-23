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


int main(int argc, char* argv[])
{
    int fd,ret;
    char *filename;
    unsigned char data;

    if(argc != 2){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}
    
    while(1){
        ret = read(fd, &data, sizeof(data));
        if(ret < 0){
            /* 数据读取错误或无效 */
        } else{
            if(data){
                printf("key value = %#X\r\n",data);
            }
        }
    }
    ret = close(fd);
    if(ret < 0)
    {
        printf("file %s close failed!\r\n",filename);
        return -1;
    }

    return 0;
}