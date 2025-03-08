/**
 * @file SHT40.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-11-14
 * @attention
 * 硬件IIC IIC1
 * PB9  SDA
 * PB8  SCL
 * @copyright Copyright (c) 2024
 *
 */
#include "SHT40.h"

extern I2C_HandleTypeDef hi2c1; // i2c的句柄变量，在i2c.h中声名

void SHT40_Read(float *humidity, float *temperature)
{
    uint8_t writeData[1] = {0xFD};
    uint8_t readData[6] = {0};
    uint32_t tempData = 0;
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SHT40_Write_ADDR, (uint8_t *)writeData, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    HAL_I2C_Master_Receive(&hi2c1, (uint16_t)SHT40_Read_ADDR, (uint8_t *)readData, 6, HAL_MAX_DELAY);
    tempData = (uint32_t)readData[0] << 8 | readData[1]; // 等价于readData[0] * 256 + readData[1])
    *temperature = (float)(tempData * 175.0f) / 65535.0f - 46;
    tempData = (uint32_t)readData[3] << 8 | readData[4];
    *humidity = (float)(tempData * 125.0f) / 65535.0f-6;
    /*特别注意，因为精度转换问题，tempData是整型的，为了保留小数精度，需要对参与运算的数据进行.0f后缀，否则将会被MDK5的C语言优化掉！！！*/
}
