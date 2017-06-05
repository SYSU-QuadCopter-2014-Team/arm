#include "SetArm.h"
#include <cstdio>

// 使用实例

char s[110];
int main() {
    printf("test\n");
    double len;
    int speed;
    
    scanf("%s", s);
    // 使用之前，先执行命令 sudo insmod /lib/modules/3.10.40/kernel/drivers/usb/serial/cp210x.ko
    // 读入串口路径，若为"-",则默认设置/dev/ttyUSB0, 不一定对。请自己找下
    // 查找命令 ls -l /dev/ttyUSB*
    if (s[0] != '-') {
        setdev(s);
    }
    
    while(1) {
        scanf("%lf%d", &len, &speed);
        double h = SetArm(len, speed);
        // len是爪子尖端之间的距离，0～29
        // speed是旋转角速度建议1～10  实际0～20
        printf("height = %lf\n", h); //没有加上底座高度，底座高度2.8cm
    }
    return 0;
}

