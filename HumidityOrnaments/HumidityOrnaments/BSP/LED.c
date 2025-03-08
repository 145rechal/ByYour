/**
 * @file LED.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-14
 * @attention
 * IO口低电平驱动LED灯
 * PB7 LED1
 * PA11 LED2
 * @copyright Copyright (c) 2024
 * 
 */
/* Includes --------------------*/
#include "LED.h"

void LED_Init(uint8_t user)
{
    user &= 0x03;//& B0000 0011,屏蔽其他位
    // & B0000 0001，取最低位
    if ((user & 0x01)!= 0)
    {
        ON_LED_1();
    }
    else
    {
        OFF_LED_1();
    }
    if ((user & 0x02)!= 0)
    {
        ON_LED_2();
    }
    else
    {
        OFF_LED_2();
    }
}
    
    

