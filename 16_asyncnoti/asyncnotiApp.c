#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "signal.h"
#include "fcntl.h"
/**
 * argc: 参数的个数
 * argv: 参数的内容
 * ./App <filename>
 * ./asyncnotiApp /dev/imx6uirq
*/

int fd;

static void sigio_signal_func(int num){
    int ret;
    unsigned int data = 0;
    ret = read(fd, &data, sizeof(data));
    if(ret < 0){
        printf("read failed\r\n");
    }else{
        printf("sigio signal, data = %d\r\n",data);
    }
}

int main(int argc, char* argv[])
{
    int ret;
    char *filename;
    int flags;

    if(argc != 2){ printf("Error Usage!\n"); return -1;}
    filename = argv[1];

    /* 打开驱动文件*/
    fd = open(filename, O_RDWR);
    if(fd < 0){printf("Can't open file %s\r\n",filename); return -1;}
    
    // while(1){
    //     ret = read(fd, &data, sizeof(data));
    //     if(ret < 0){
    //         /* 数据读取错误或无效 */
    //     } else{
    //         if(data){
    //             printf("key value = %#X\r\n",data);
    //         }
    //     }
    // }
    /* 设置信号处理函数 */
    signal(SIGIO, sigio_signal_func);

    fcntl(fd, F_SETOWN, getpid());      /* 设置当前进程接收SIGIO信号 */
    flags = fcntl(fd, F_GETFL);         /* 获取当前进程的状态 */
    fcntl(fd, F_SETFL, flags | FASYNC); /* 开启当前进程异步通知的功能 */

    while(1){
        sleep(2);
    }
    ret = close(fd);
    if(ret < 0)
    {
        printf("file %s close failed!\r\n",filename);
        return -1;
    }

    return 0;
}