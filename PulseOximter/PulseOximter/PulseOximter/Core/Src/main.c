/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>

#include "lcd.h"
#include "ugui.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*GUI相关参数*/
UG_GUI gui;
#define MAX_OBJECTS 10
UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];
UG_OBJECT obj_buff_wnd_2[MAX_OBJECTS];

UG_WINDOW window_1;
UG_BUTTON button_1;
UG_TEXTBOX textbox_1;

UG_WINDOW window_2;

/*ADC相关参数*/

/*电池电量*/
uint16_t POWER = 40; // 保存电池电量（初始值=80）

/*SPI相关参数*/
uint8_t i, m;
float t = 0;

/*按键相关参数*/
uint8_t flag = 0;
uint8_t lastflag = 0;
uint8_t KEY_state = 0; // 按键状态(0 = 未按下；1 = 按下）
uint16_t KEY_cnt = 0;  // 按键时长累加器

/*128次采样周期结束标志：1为采样中，0为采样结束*/
uint8_t sample_flag = 0;

/*定时器相关参数*/
uint32_t TIM1_counter3 = 3; /*控制发射管的启停时序*/
uint32_t TIM1_counter1 = 0; /*控制按键时序*/

/*发射管引脚状态(共4种状态，初始为状态0)*/
uint8_t SEND_status = 0;

/*发射管电流控制*/
uint8_t pulse_IRcurrent = 0;
uint8_t pulse_Rcurrent = 0;

/*发射管电流增强值(最大为699*/
uint8_t pulse_current_enhance = 300;

/*存储当前处理的信号类型标志：0表示红外，1表示红光*/
uint8_t IRorRED = 0;

/*系统运行状态(0=开机，1=关机)*/
uint8_t system_status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void window_1_callback(UG_MESSAGE *msg)
{
  if (msg->type == MSG_TYPE_OBJECT)
  {
    if (msg->id == OBJ_TYPE_BUTTON)
    {
      switch (msg->sub_id)
      {
      case BTN_ID_0:
        UG_WindowShow(&window_2);
        /* code */
        break;
      case BTN_ID_1:
        /* code */
        break;
      case BTN_ID_2:
        /* code */
        break;
      case BTN_ID_3:
        /* code */
        break;
      case BTN_ID_4:
        /* code */
        break;
      case BTN_ID_5:
        /* code */
        break;
      case BTN_ID_6:
        /* code */
        break;
      case BTN_ID_7:
        /* code */
        break;
      case BTN_ID_8:
        /* code */
        break;
      case BTN_ID_9:
        /* code */
        break;
      }
    }
  }
}

void First_init(void)
{
  /*开始界面初始化*/

  UG_Init(&gui, (void (*)(UG_S16, UG_S16, UG_COLOR))LCD_DrawPoint, LCD_W, LCD_H); // uGUI初始化
  UG_SelectGUI(&gui);
  UG_WindowCreate(&window_1, obj_buff_wnd_1, MAX_OBJECTS, window_1_callback); // 创建窗口1

  UG_WindowSetTitleText(&window_1, "Pulse Oximeter");
  UG_WindowSetTitleTextFont(&window_1, &FONT_8X8);
  UG_WindowSetBackColor(&window_1, WHITE);
  UG_WindowSetTitleColor(&window_1, GREEN);
  UG_WindowSetTitleTextColor(&window_1, BLACK);

  UG_ButtonCreate(&window_1, &button_1, BTN_ID_0, 36, 0, 126, 35);
  UG_ButtonSetFont(&window_1, BTN_ID_0, &FONT_8X12);
  UG_ButtonSetForeColor(&window_1, BTN_ID_0, RED);
  UG_ButtonSetBackColor(&window_1, BTN_ID_0, BLACK);
  UG_ButtonSetText(&window_1, BTN_ID_0, "Measure");
  UG_ButtonSetStyle(&window_1, BTN_ID_0, BTN_STYLE_3D);
  // UG_TextboxCreate(&window_1, &textbox_1, TXB_ID_0, 0, 0, 126, 30);
  // UG_TextboxSetFont(&window_1, TXB_ID_0, &FONT_6X8);
  // UG_TextboxSetForeColor(&window_1, TXB_ID_0, BLACK);
  // UG_TextboxSetBackColor(&window_1, TXB_ID_0, GREEN);
  // UG_TextboxSetText(&window_1, TXB_ID_0, "AAA");

  UG_WindowShow(&window_1);
  UG_WindowResize(&window_1, 0, 0, 159, 57);
  UG_SetForecolor(BLUE);
  UG_SetBackcolor(WHITE);
  UG_FontSelect(&FONT_12X20);
  UG_PutString(0, 60, "BATTERY:");

  LCD_ShowBattey(100, 56, 1);
  LCD_Show_Power(100, 56, 1, POWER);
}
void window_2_callback(UG_MESSAGE *msg)
{
  if (msg->type == MSG_TYPE_OBJECT)
  {
    if (msg->id == OBJ_TYPE_BUTTON)
    {
      switch (msg->sub_id)
      {
      case BTN_ID_0:
        UG_WindowDelete(&window_2);
        First_init();
        /* code */
        break;
      case BTN_ID_1:
        /* code */
        break;
      case BTN_ID_2:
        /* code */
        break;
      case BTN_ID_3:
        /* code */
        break;
      case BTN_ID_4:
        /* code */
        break;
      case BTN_ID_5:
        /* code */
        break;
      case BTN_ID_6:
        /* code */
        break;
      case BTN_ID_7:
        /* code */
        break;
      case BTN_ID_8:
        /* code */
        break;
      case BTN_ID_9:
        /* code */
        break;
      }
    }
  }
}

void TFT_MENU(void)
{
  if (flag == 1)
  {
    UG_MESSAGE msg;
    msg.type = MSG_TYPE_OBJECT;
    msg.id = OBJ_TYPE_BUTTON;
    msg.sub_id = BTN_ID_0;
    // 将消息发送到处理函数或队列
    // 这里需要根据您的系统架构来实现消息的传递
    window_1_callback(&msg);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
  if (flag == 2)
  {
    UG_MESSAGE msg;
    msg.type = MSG_TYPE_OBJECT;
    msg.id = OBJ_TYPE_BUTTON;
    msg.sub_id = BTN_ID_0;
    // 将消息发送到处理函数或队列
    // 这里需要根据您的系统架构来实现消息的传递
    window_2_callback(&msg);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}

void Measuring_init(void)
{
  /*Measuring界面初始化*/
  UG_WindowCreate(&window_2, obj_buff_wnd_2, MAX_OBJECTS, window_2_callback); // 创建窗口2

  UG_WindowSetTitleText(&window_2, "Pulse Oximeter");
  UG_WindowSetTitleTextFont(&window_2, &FONT_6X8);
  UG_WindowResize(&window_2, 10, 10, 140, 75);
  UG_WindowSetBackColor(&window_2, WHITE);
  UG_WindowSetTitleColor(&window_2, GREEN);
  UG_WindowSetTitleTextColor(&window_2, BLACK);

  UG_WindowSetTitleInactiveColor(&window_1, C_BLACK);
  UG_WindowSetTitleInactiveTextColor(&window_1, C_CYAN);

  UG_TextboxCreate(&window_2, &textbox_1, TXB_ID_0, 0, 0, 126, 40);
  UG_TextboxSetFont(&window_2, TXB_ID_0, &FONT_8X8);
  UG_TextboxSetForeColor(&window_2, TXB_ID_0, BLACK);
  UG_TextboxSetBackColor(&window_2, TXB_ID_0, GREEN);
  UG_TextboxSetText(&window_2, TXB_ID_0, "Measuring...");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

  Lcd_Init(); // 初始化OLED
  LCD_Clear(WHITE);
  BACK_COLOR = WHITE;
  First_init();
  Measuring_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Measuring_init();
    if (flag != lastflag)
    {
      // 按键状态变化时调用TFT_MENU函数，并传递当前状态
      TFT_MENU();
      // 更新lastButtonState
      lastflag = flag;
    }
    UG_Update();
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // 点亮LED
    // HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    KEY_state = 1;
  }
}
/*控制时序说明：每次发射包括四个阶段（IR发射、停止发射、RED发射、停止发射）,每阶段3ms，共计12ms；之后为27ms的延迟（停止发射）；
          上述为一个完整发射循环，每个循环为39ms；完整发射周期包括128次发射循环，共计4.992秒。
  发射管控制时序：
              |IR发射|停止发射|RED发射|停止发射| 延迟 |IR发射|停止发射|RED发射|停止发射| 延迟 |......
    ---------------------------------------------------------------------------------------------------------
      时序(ms)        0      3        6       9        12     39     42       45      48       51     78......
    ---------------------------------------------------------------------------------------------------------
      SEND_state 	    |  1   |   1    |   1   |   1    |   0  |  1   |   1    |   1   |   1    |   0  |......
    ---------------------------------------------------------------------------------------------------------
      SEND_status     |  0   |   1    |   2   |   3    |   0  |  0   |   1    |   2   |   3    |   0  |......
    ---------------------------------------------------------------------------------------------------------
      BTIM1_counter1 △								 △	                                     △
    ---------------------------------------------------------------------------------------------------------
      BTIM1_counter2                                   △                                      △
    ---------------------------------------------------------------------------------------------------------
      pulse_start    △     △       △      △        △ ... △     △       △      △       △ ... △
    ---------------------------------------------------------------------------------------------------------

  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim == &htim3)
  {
    if (sample_flag == 1)
    {
      if (TIM1_counter3 > 2) /*计数达到3ms*/
      {
        TIM1_counter3 = 0;
        switch (SEND_status)
        {
        case 0: /*发射红外信号<3ms*/
        {
          SEND_status++;
          HAL_GPIO_WritePin(IRLED_GPIO_Port, IRLED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, GPIO_PIN_SET);
          pulse_Rcurrent = 0;
          pulse_IRcurrent = 300 + pulse_current_enhance;
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_IRcurrent); // 设置红外占空比为300+调整值(因为自动重装载值是1000)
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_Rcurrent);  // 设置红光占空比为0；
          HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
          HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
          IRorRED = 0; /*红外标志*/
          /*启动ADC采样*/
          break;
        }
        case 1: /*关闭信号发射<3ms*/
        {
          SEND_status++;
          HAL_GPIO_WritePin(IRLED_GPIO_Port, IRLED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, GPIO_PIN_RESET);
          pulse_Rcurrent = 0;
          pulse_IRcurrent = 0;
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_IRcurrent); // 设置红外占空比为0
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_Rcurrent);  // 设置红光占空比为0
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
          break;
        }
        case 2: /*发射红光信号<3ms*/
        {
          SEND_status++;
          HAL_GPIO_WritePin(IRLED_GPIO_Port, IRLED_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, GPIO_PIN_RESET);
          pulse_IRcurrent = 0;
          pulse_Rcurrent = 300 + pulse_current_enhance;
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_IRcurrent);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_Rcurrent);
          HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
          HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
          IRorRED = 1; /*红光标志*/
          /*启动ADC采样*/
          break;
        }
        case 3: /*关闭信号发射<3ms*/
        {
          SEND_status++;
          HAL_GPIO_WritePin(IRLED_GPIO_Port, IRLED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, GPIO_PIN_RESET);
          pulse_Rcurrent = 0;
          pulse_IRcurrent = 0;
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_IRcurrent); // 设置红外占空比为0
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_Rcurrent);  // 设置红光占空比为0
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
          break;
        }
        }
      }
      else
      {
        TIM1_counter3++; /*计数器累加*/
      }
    }
    /*按键控制*/
    if (KEY_state == 1) /*按键状态为按下*/
    {
      if (TIM1_counter1 > 10) /*每隔10ms检测一次*/
      {
        TIM1_counter1 = 0;
        if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
        {
          KEY_cnt++; /*按键时长累加器*/
        }
        else/*松开状态*/
        {
          if (KEY_cnt < 200)/*按下时间小于2秒，短按*/
          {
            KEY_cnt = 0;
            TIM1_counter1 = 0;
            KEY_state = 0;
            if (flag < 2)
            {
              flag++;
            }
            else
            {
              flag = 0;
            }
          }
          if (KEY_cnt >= 200)/*按下时间大于2秒，长按*/
          {
            KEY_cnt = 0;
            TIM1_counter1 = 0;
            KEY_state = 0;
            if (system_status==0)/*系统运行状态为开机*/
            {
              system_status = 1;/*关机*/
            }
            else
            {
              system_status = 0;/*开机*/
            }
          }
        }
      }
      else
      {
        TIM1_counter1++; /*计数器累加*/
      }
    }
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
