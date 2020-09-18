/*

本节点为底层串口设备的参数配置及读写

*/

#include "serialport.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


static void set_baudrate(struct termios *opt,unsigned int baudrate)
{
    cfsetspeed(opt, baudrate);
    opt->c_cflag |= CLOCAL | CREAD;//允许接受，本地模式
}

static void set_data_bit(struct termios *opt,unsigned int databit)
{
    opt->c_cflag &= ~CSIZE;
    switch (databit)
    {
    case 8:
        opt->c_cflag |= CS8;
        break;
    case 7:
        opt->c_cflag |= CS7;
        break;
    case 6:
        opt->c_cflag |= CS6;
        break;
    case 5:
        opt->c_cflag |= CS5;
        break;
    default:
        opt->c_cflag |= CS8;
        break;
    }
}

static void set_parity(struct termios *opt,char parity)
{
    switch (parity) {
    case 'N': /* 无校验 */
    case 'n':
        opt->c_cflag &= ~PARENB;
        break;
    case 'E': /* 偶校验 */
    case 'e':
        opt->c_cflag |= PARENB;
        opt->c_cflag &= ~PARODD;
        break;
    case 'O': /* 奇校验 */
    case 'o':
        opt->c_cflag |= PARENB;
        opt->c_cflag |= ~PARODD;
        break;
    default: /* 其它选择为无校验 */
        opt->c_cflag &= ~PARENB;
        break;
    }
}

static void set_stopbit(struct termios *opt,const char *stopbit)
{
    if (0 == strcmp (stopbit, "1")) {
        opt->c_cflag &= ~CSTOPB; /* 1 位停止位 t */
    } else if (0 == strcmp (stopbit, "1.5")) {
        opt->c_cflag &= ~CSTOPB; /* 1.5 位停止位 */
    } else if (0 == strcmp (stopbit, "2")) {
        opt->c_cflag |= CSTOPB; /* 2 位停止位 */
    } else {
        opt->c_cflag &= ~CSTOPB; /* 1 位停止位 */
    }
}

static int set_port_attr(int fd,int baudrate,int databit,char parity,const char *stopbit,int vtime,int vmin)
{
    struct termios opt;
    tcgetattr(fd, &opt);

    set_baudrate(&opt, baudrate);//设置波特率
    set_data_bit(&opt, databit);//设置数据位宽度
    set_parity(&opt, parity);//设置奇偶校验位
    set_stopbit(&opt, stopbit);//设置停止位


    opt.c_oflag = 0;//清空输出属性
    opt.c_oflag &= ~OPOST;//输出属性设置为原始输出

    opt.c_lflag = ~ICANON;//原始模式(取消行缓冲)

    opt.c_cc[VTIME] = vtime;//非标准模式:读操作延时
    opt.c_cc[VMIN] = vmin;//非标准模式:读的最小字符数
    tcflush (fd, TCIFLUSH);
    return (tcsetattr (fd, TCSANOW, &opt));
}


int openSerialPort(const char *dev)
{
    int fd = open(dev,O_RDWR | O_NOCTTY);
    if(fd < 0)
    {
        perror("open uart device error \n");
        return fd;
    }
    int ret = set_port_attr(fd,B9600,8,'N',"1",150,255);
    if(ret < 0)
    {
        printf("set uart arrt failed \n");
        return -1;
    }
    return fd;
}

void closeSerialPort(int fd)
{
    close(fd);
}



int SendData(int fd,char* data,unsigned int length)
{
    return write(fd,data,length);
}

int RecvData(int fd,char* data,unsigned int length)
{
    return read(fd,data,length);
}
