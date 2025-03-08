#ifndef __LED_H
#define __LED_H

#include "main.h"

#define TOG_LED_1() HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin)
#define TOG_LED_2() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
#define ON_LED_1() HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET)
#define ON_LED_2() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define OFF_LED_1() HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET)
#define OFF_LED_2() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
void LED_Init(uint8_t user);

#endif /* __LED_H */
