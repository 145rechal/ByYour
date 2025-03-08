/**
  ******************************************************************************
  * @file    adc.h
  * @author  深圳市沃瑞珂科技有限公司--嵌入式开发团队
             SHENZHEN WRC Application Team
  * @version V1.0.0
  * @date    2022-04-19
  * @brief   CHY-6YH-JK板子的adc相关操作与初始化
  ******************************************************************************
  */
#ifndef __MCUTEMP_H__
#define __MCUTEMP_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdarg.h>
#include <stdio.h>
#include "main.h"

#define CSD_NUMBER 1
#define TEMP_NUMBER 2

    /**
     * @brief  CDS ADC Init.
     * @param  None
     * @retval None
     */
    void CDS_ADC_Init(void);
    uint16_t Get_Adc_value(uint8_t ch);
    float Get_MCU_Temperature(void);

#ifdef __cplusplus
}
#endif
#endif
