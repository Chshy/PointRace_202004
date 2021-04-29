#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int fd;           //串口句柄

//设置串口参数
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;

    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));   //newtio清零
    newtio.c_cflag |= CLOCAL | CREAD; //忽略调制解调器线路状态 | 使用接收器
    newtio.c_cflag &= ~CSIZE;         //屏蔽字符大小位

    //数据位
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    //波特率 默认9600
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 100; ///* 设置超时10 seconds*/
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }

    return 0;
}

void pi_uart_init(int BaudRate=9600)
{
    char *uart3 = "/dev/ttyAMA0"; //已将ttyAMA0从蓝牙串口改为硬件串口
    
    //打开串口
    //if((fd = open(uart3, O_RDWR|O_NOCTTY))<0)//默认为阻塞读方式
    if ((fd = open(uart3, O_RDWR | O_NONBLOCK)) < 0) //非阻塞读方式
        printf("open %s is failed", uart3);
    else
    {
        set_opt(fd, BaudRate, 8, 'N', 1); //设置串口参数
    }
    return;
}

void pi_uart_write(char *TxBuf,int Txlen = -1)
{
    if(Txlen==-1)
    {
        Txlen=strlen(TxBuf);
    }
    write(fd, TxBuf, Txlen);
    return;
}

void pi_uart_read(char *RxBuf, int Rxlen = 512)
{
    int nByte = 0;
    memset(RxBuf, 0, strlen(RxBuf));

    nByte = read(fd, RxBuf, Rxlen);
    if (nByte <= 0)
    {
        *RxBuf = '\0';
    }
    else
    {
        RxBuf[nByte + 1] = '\0';
    }
    return;
}
