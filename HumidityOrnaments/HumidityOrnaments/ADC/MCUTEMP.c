/**
  ******************************************************************************
  * @file    pwm.c
  * @author  深圳市沃瑞珂科技有限公司--嵌入式开发团队
             SHENZHEN WRC Application Team
  * @version V1.0.0
  * @date    2022-04-19
  * @brief   CHY-6YH-JK板子的adc相关操作与初始化
  ******************************************************************************
  */
#include "MCUTEMP.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

#define ADC_VALUE_NUM 32
#define ADC_CHANNEL_NUM 5

__IO uint16_t adcCovValueBuff[ADC_VALUE_NUM][ADC_CHANNEL_NUM] = {0}; // 存放ADC的值ADC_CHANNEL_NUM通道  每个通道存放30个值，由DMA循环写入
uint16_t adcAverageBuff[ADC_CHANNEL_NUM] = {0};                      // 对每个通道30个ADC值取平均值

/**
 * @brief  CDS ADC Init.
 * @param  None
 * @retval None
 */
void CDS_ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1); // 校准ADC
    HAL_Delay(100);                     // 延时100ms
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcCovValueBuff, ADC_VALUE_NUM * ADC_CHANNEL_NUM);
}

// 获得ADC值
// ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
// 返回值:转换结果
uint16_t Get_Adc_value(uint8_t ch)
{
    uint32_t sum = 0;

    if ((ch > ADC_CHANNEL_NUM) || (ch == 0))
        return -1;

    for (uint8_t count = 0; count < ADC_VALUE_NUM; count++)
    {
        sum += adcCovValueBuff[count][ch - 1];
    }

    adcAverageBuff[ch - 1] = sum / ADC_VALUE_NUM;
    sum = 0;

    return adcAverageBuff[ch - 1];
}

float Get_MCU_Temperature(void)
{
    uint32_t sum = 0;
//    float Vol_Value = 0;
    float Temperature = 0;
    uint8_t TEMP_CH;
    uint16_t TS_CAL1;

    TEMP_CH = TEMP_NUMBER;

    if ((TEMP_CH > ADC_CHANNEL_NUM) || (TEMP_CH == 0))
        return -1;

    for (uint8_t count = 0; count < ADC_VALUE_NUM; count++)
    {
        sum += adcCovValueBuff[count][TEMP_CH - 1];
    }

    adcAverageBuff[TEMP_CH - 1] = sum / ADC_VALUE_NUM;

    TS_CAL1 = *(__IO uint16_t *)(0x1FFF75A8);
    Temperature = (30.0f) / (TS_CAL1) * ((float)adcAverageBuff[TEMP_CH - 1] - TS_CAL1) + 30.0f;

    return Temperature;
}
