/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
// 定义宏来�?�? printf 切换和输�?
#define PRINTF_PC(...)       \
  do                         \
  {                          \
    current_huart = &huart1; \
    printf(__VA_ARGS__);     \
  } while (0)

// #define PRINTF_HC05(...)     \
//   do                         \
//   {                          \
//     current_huart = &huart2; \
//     printf(__VA_ARGS__);     \
//   } while (0)

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
// #define RX_CMD_LEN 5//表示5字节的指令长度
// extern  uint8_t rxBuffer[];
// extern uint8_t isUploadTime;//标志位，表示是否上传时间数据
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
// void on_UART_IDLE(UART_HandleTypeDef *huart);
// void updateRTCData(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

