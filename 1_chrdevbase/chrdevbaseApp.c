#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

/**
 * 文件名    :chrdevbaseApp.c
 * 作者      ：azumid
 * 版本      :V1.0
 * 描述      ：chrdevbase驱动测试APP
 * 其他      ：使用方法： ./chrdevbaseApp /dev/chrdevbase <1>|<2>
 *              argv[2] 1:读文件
 *              argv[2] 2:写文件
*/


static char usrdata[] = {"usr data!"};

/**
 * @description    : main主程序
 * @param - argc   : argv数组元素个数
 * @param - argv   : 具体参数
 * @return         : 0 成功；其他 失败
*/

int main(int argc, char* argv[])
{
    int fd,ret;
    char *filename;
    char readbuf[100],writebuf[100];

    if(argc != 3){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}

    if(atoi(argv[2]) == 1)
    {
        ret = read(fd, readbuf, 50);
        if(ret < 0) {printf("read file %s failed!\r\n",filename);}
        else 
        {
            printf("read data: %s\r\n",readbuf);
        }
    }
    if(atoi(argv[2]) == 2)
    {
        memcpy(writebuf, usrdata, sizeof(usrdata));
        ret = write(fd, writebuf, 50);
        if(ret < 0) printf("write file %s failed!\r\n",filename);

    }

    // ret = close(fd);
    // if(ret < 0) {printf("Can't close file %s\r\n",filename); return -1;}
    return 0;
}