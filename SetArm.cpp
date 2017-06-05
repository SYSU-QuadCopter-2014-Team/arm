#include "SetArm.h"


#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>
#include     "string.h"
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/

#include     <math.h>


#define FALSE  -1
#define TRUE   0
/*********************************************************************/
int OpenDev(char *Dev)
{
    int fd = open(Dev, O_RDWR | O_NOCTTY | O_NDELAY);         //| O_NOCTTY | O_NDELAY
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
        return fd;
}
/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
*/
int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,
                    19200,  9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
        if  (speed == name_arr[i]) {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if  (status != 0) {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}
/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    if  ( tcgetattr( fd,&options)  !=  0) {
        perror("SetupSerial 1");
        return(FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size/n"); return (FALSE);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;   /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            options.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;     /* Enable parity */
            options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
            options.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        case 'S':
        case 's':  /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parity/n");
            return (FALSE);
    }
    /* 设置停止位*/
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
           break;
        default:
             fprintf(stderr,"Unsupported stop bits/n");
             return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

char dev[110] = "/dev/ttyUSB0";

void setdev(char s[]) {
    int n = strlen(s);
    for (int i = 0; i <= n; i++) dev[i] = s[i];
}

void SerArmSpeed(int speed) {
    if (speed < 0 || speed > 20) {
        perror("speed error!\n");
        return -1;
    }

    int fd;
    int nread;
    char buff[11] = {0};

    buff[0] = 0xff;
    buff[1] = 0x01;
    buff[2] = 0x00;
    buff[3] = 0xff&speed;
    buff[4] = speed>>8;

    fd = OpenDev(dev);
    set_speed(fd, 9600);
    if (set_Parity(fd,8,1,'n') == FALSE)
    {
        printf("Set Parity Error/n");
        return -1;
    }
    nread = write(fd, buff, 11);
    close(fd);

    if (nread <0) {
        printf("speed write failed!\n");
    } else {
        printf("speed write success\n");
    }

}

void SetArmbyAngle(int theta, int speed) {
    SerArmSpeed(speed);
    if (theta <0 || theta > 180) {
        printf("angle is illegle\n");
        return ;
    }

    int fd;
    int nread;
    char buff[11] = {0};

    buff[0] = 0xff;
    buff[1] = 0x02;
    buff[2] = 0x00;
    buff[3] = 0xff&theta;
    buff[4] = theta >> 8;

    fd = OpenDev(dev);
    set_speed(fd, 9600);
    if (set_Parity(fd,8,1,'n') == FALSE) {
        printf("Set Parity Error/n");
        return;
    }
    nread = write(fd, buff, 11);
    close(fd);

    if (nread <0) {
        printf("angle write failed!\n");
    } else {
        printf("angle write success\n");
    }

}

double SetArmbyLen(double len, int speed) {
    if (len < 0 || len >29) {
        perror("len error!\n");
        return -1;
    }

    int fd;
    int nread;
    char buff[11] = {0};
    
    double l = 13.5;
    double m = 2.5;
    double theta = asin(1.0*(len-m)/2/l);
    int _theta = theta / acos(-1.0) * 180;

    _theta = 120 - _theta;
    _theta = (_theta+45)/0.09;

    SetArmbyAngle(_theta, speed);

    return cos(theta) * l;
}

