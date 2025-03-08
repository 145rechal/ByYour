#ifndef __SN74HC595_H
#define __SN74HC595_H

#include "main.h"

#define DIS_DOT 0x20
#define DIS_BLACK 0x10
#define DIS_ 0x11

/*SER PA5*/
#define SER_L() HAL_GPIO_WritePin(SMG_SER_GPIO_Port, SMG_SER_Pin, GPIO_PIN_RESET)
#define SER_H() HAL_GPIO_WritePin(SMG_SER_GPIO_Port, SMG_SER_Pin, GPIO_PIN_SET)

/*RCK PB2*/
#define RCK_L() HAL_GPIO_WritePin(SMG_RCK_GPIO_Port, SMG_RCK_Pin, GPIO_PIN_RESET)
#define RCK_H() HAL_GPIO_WritePin(SMG_RCK_GPIO_Port, SMG_RCK_Pin, GPIO_PIN_SET)

/*SCK PB0*/
#define SCK_L() HAL_GPIO_WritePin(SMG_SCK_GPIO_Port, SMG_SCK_Pin, GPIO_PIN_RESET)
#define SCK_H() HAL_GPIO_WritePin(SMG_SCK_GPIO_Port, SMG_SCK_Pin, GPIO_PIN_SET)

void SN74HC595_Write(uint8_t data);
void SN74HC595_Out(void);
void SN74HC595_RST(void);
void LED6_RST(void);
void Display_Scan(void);
#endif
