#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "poll.h"
#include "sys/select.h"
#include "sys/time.h"
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


int main(int argc, char* argv[])
{
    int fd,ret;
    char *filename;
    unsigned char data;
    struct pollfd fds;
    fd_set readfds;
    struct timeval timeout;

    if(argc != 2){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR | O_NONBLOCK);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}
#if 0
    /* 构造结构体 */
    fds.fd = fd;
    fds.events = POLLIN;
    while(1){

        ret = poll(&fds, 1, 500);
        if(ret){
            ret = read(fd, &data, sizeof(data));
            if(ret < 0){
            /* 读取错误 */
            }else{
                if(data)
                printf("key value = %d \r\n", data);
            }
        } else if(ret == 0){    /* 超时 */

        } else if(ret < 0){     /* 错误 */
        
        }
        
    }
#endif

    while(1){
       FD_ZERO(&readfds);
       FD_SET(fd, &readfds);
       /* 构造超时时间 */
       timeout.tv_sec = 0;
       timeout.tv_usec = 500000;    /* 500ms */
       ret = select(fd+1, &readfds, NULL, NULL, &timeout);
       switch(ret){
        case 0: /* 超时 */
            /* 用户自定义超时处理 */
            break;
        case -1:    /* 错误 */
            break;
        default:
            if(FD_ISSET(fd, &readfds)){
                ret = read(fd, &data, sizeof(data));
                if(ret < 0){
                    /* 读取错误 */
                } else{
                    if(data)
                        printf("key value = %d\r\n",data);
                }
            }
            break;
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