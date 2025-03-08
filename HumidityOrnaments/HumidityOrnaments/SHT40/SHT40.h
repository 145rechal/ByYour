#ifndef __SHT40_H
#define __SHT40_H

#include "main.h"
// 由数据手册可知，0x44是IIC地址；当地址最低位是0，表示读数据，最低位是1，表示写数据；
#define SHT40_Write_ADDR (0x44 << 1)      // SHT40地址，默认是7位，发送是8位，需要进行左移，然后加入发送与读取指令在最后一位
#define SHT40_Read_ADDR ((0x44 << 1) + 1) // 最后一位是1是读
/*虽然HAL库代码中已经给我们自动添加了，这里我们手动也填充一下，防止后续移植出问题。*/
#define SHT40_MEASURE_TEMPERATURE_HUMIDITY 0XFD // 高精度读取温湿度命令
#define SHT40_READ_SERIAL_NUMBER 0x89           // 读取唯一序列号命令
#define SHT40_HEATER_200mW 0x39                 // 200mW加热1s命令
void SHT40_Read(float *humidity, float *temperature);
#endif
