/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "Key.h"
#include "stdio.h" //包含printf函数的头文件
#include <string.h>
#include "SHT40.h"
#include "SN74HC595.h"
#include "MCUTEMP.h"
#include <math.h>
#include "ws2812.h"

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
/*
 * TIM3: us延时函数
 * TIM14: 100Hz任务调度
 * TIM16: 1000Hz数码管刷新
 */
/*i2c1*/
float temperature, humidity;
/*TIM14*/
uint8_t flag_TIM14;   // 100HZd==0.01s==10ms update interrupt
uint8_t EVENT_500ms;  // 500ms==50*flag_TIM1 update SMG
uint8_t EVENT_700ms;  // 700ms==70*flag_TIM1 Read SHT40
uint8_t EVENT_900ms;  // 900ms==90*flag_TIM1 print all DATA
uint8_t EVENT_1100ms; // 1100ms==110*flag_TIM1 SLPEE PWR_SLEEP
/*SN74HC595*/
uint8_t LED6[6];             // 显示缓存
uint8_t display_index;       // 显示位索引
uint16_t SIG_tem = 0;        // display temperature £¬ÐèÒªÈ¡Î»£¬%£¬ÐèÒªÕûÐÍ
uint16_t SIG_hum = 0;        // display humidity
uint16_t SIG_vol = 0;        // display voltage
uint16_t voltagePercent = 0; // 电压百分比
uint8_t display_flag = 0x00; // 显示标志位
/*KEY*/
uint8_t KEY_EVENT = 0x00; // 按键事件
/*ADC*/
#define INTERVREFINT 1.22       // 参考电压
#define CHANNELNUM 5            // 通道5
uint16_t ADC_value[CHANNELNUM]; // DMA传输ADC采集值变量内存数量
uint16_t VREFINT_CAL;
/*
*cubemx中的Rank就是DMA的通道对应的ADC通道
[0]:ADC1_IN6 PA6 光强
[1]:ADC1_IN9 PB1 电池电压
[2]:Temperature Sensor Channel
[3]:Vrefint Channel
[4]:Vbat Channel
*/
float voltageValue[CHANNELNUM];
float vrefint = 0;
float mcutem = 0;

/*SLEEP*/
// 功能设计：开机默认运行3500ms(保证运行3次EVENT_1100ms事件)
//--->关闭TIM0,TIM1定时中断，停止ADC_DMA(SLEEP模式外设仍工作，手动停止)，关闭SYSTICK中断，进入PWR_SLEEP模式，关灯，设置挡位错开
//--->控制SN74HC595的输出，使得LED6的显示全部关闭
//--->等待WFI触发退出SLEEP，恢复系统时钟中断，恢复TIM0,TIM1定时中断，开启ADC_DMA，开启SYSTICK中断，打开LED，设置挡位正常
#define SLEEP_COUNTDOWN 10 // 睡眠倒计时，或说睡眠前显示的倒计回合数
uint8_t flag_SLEEP = 0;    // 睡眠标志
uint8_t EVENT_1100ms_cnt = 0;

/*BULETOOTH*/
uint8_t receiveData[50];   // 接收数据
char txData[] = "AT+VERS"; // 数据
uint8_t SUM = 0;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

/*KEYCHANGE*/
typedef enum
{
  MODE1,
  MODE2,
  MODE3
} DisplayMode;

DisplayMode currentMode = MODE1;

/*RTC*/
// uint8_t TimeStr[20];

// uint8_t proBuffer[10];
// uint8_t rxBuffer[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*__RE-CODE printf()__*/
UART_HandleTypeDef *current_huart = &huart1;
int fputc(int ch, FILE *f)
{
  uint8_t temp[1] = {ch};
  HAL_UART_Transmit(current_huart, temp, 1, 2);
  return ch;
}

/*TIM*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim14.Instance)
  {
    flag_TIM14 = 1;
    if (EVENT_1100ms_cnt >= SLEEP_COUNTDOWN)
    {
      flag_SLEEP = 1;
    }
  }
  if (htim->Instance == htim16.Instance)
  {
    Display_Scan();
  }
}

/*WEAK(注意只有一个中断线)*/
// 0x01:KEY_WAKEUP_GPIO_Port, KEY_WAKEUP_Pin, KEY_WAKEUP_EXTI_IRQn  -----EXIT line15 interrupt PB5
// 0x02:KEY_GPIO_Port, KEY_Pin, KEY_UP_EXTI_IRQn  -----EXIT line7 interrupt PA7
#define KEY_EVENT_WAKE_STATUS() HAL_GPIO_ReadPin(WAKEUP_GPIO_Port, WAKEUP_Pin) // 消抖标志位
// #define KEY_EVENT_STATUS() HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)            // 消抖标志位

/*按内部电压转换各个通道的校准值*/
void convertCALvoltage(void)
{
  uint8_t i;
  vrefint = INTERVREFINT / ADC_value[3]; // 首先读取VREF的电压对应的ADC值，又已知STM32F103C8T6内部电压值默认为1.2V，因此读取到的数值和默认值做比例算术，可以获得一个比值，这个比例将作为其他通道的换算比例，实现使用内部参考电压校准其他ADC采样通道
  for (i = 0; i < CHANNELNUM; i++)
  {
    /* code */
    voltageValue[i] = (float)ADC_value[i] * vrefint; // 校准ADC采样值，得到实际电压
    mcutem = (ADC_value[2] - 1042) * (1 / 2.5) + 30;
    double absolute_value = fabs(mcutem);
    mcutem = (float)absolute_value;
  }
}

uint16_t Get_VREFINT_CAL(void)
{
  return *(__IO uint16_t *)(0x1FFF75A8);
}

void SMG_Display(uint8_t display_flag)
{
  uint16_t SIG_tem = 0; // display temperature
  uint16_t SIG_hum = 0; // display humidity
  // uint16_t SIG_vol = 0; // display voltage
  switch (display_flag)
  {
  case 0x00: // 显示温湿度
    SIG_tem = temperature * 10;
    SIG_hum = humidity * 10;
    LED6[3] = (uint8_t)(SIG_tem / 100);
    LED6[4] = (uint8_t)((SIG_tem % 100) / 10);
    LED6[4] |= 0x20; // 显示小数点
    LED6[5] = (uint8_t)(SIG_tem % 10);

    LED6[0] = (uint8_t)(SIG_hum / 100);
    LED6[1] = (uint8_t)((SIG_hum % 100) / 10);
    LED6[1] |= 0x20; // 显示小数点
    LED6[2] = (uint8_t)(SIG_hum % 10);
    break;
  case 0x01: // 显示电量和光强
    LED6[3] = SIG_vol / 100;
    LED6[3] |= 0x20; // 显示小数点
    LED6[4] = SIG_vol / 10 % 10;
    LED6[5] = SIG_vol % 10;

    LED6[0] = (uint8_t)(voltagePercent / 100);
    LED6[1] = (uint8_t)((voltagePercent % 100) / 10);
    LED6[1] |= 0x20; // 显示小数点
    LED6[2] = (uint8_t)(voltagePercent % 10);
    // LED6[0] = (uint8_t)(voltageValue[0] / 1000);
    // LED6[1] = (uint8_t)(voltageValue[0] / 100 % 10);
    // LED6[2] = (uint8_t)(voltageValue[0] / 10 % 10);
    break;
  case 0x02: // 显示上位机发送的时间
    // LED6[3] = 0x10;
    // // LED6[4] = proBuffer[0];
    // // LED6[4] |= 0x20; // 显示小数点
    // LED6[5] = proBuffer[1];

    // LED6[0] = 0x10;
    // LED6[1] = proBuffer[2];
    // LED6[2] = proBuffer[3];
    break;
  default:
    LED6_RST();
    break;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart2)
  {
    HAL_UART_Transmit_DMA(&huart2, receiveData, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData)); // 串口不定长数据接收
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);                        // 关闭DMA接收过半中断
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t i;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // 初始化LED
  LED_Init(0x00);
  // 初始化串口
  // uint8_t hello1[] = "Hello,blocking\n";
  // HAL_UART_Transmit(&huart1, hello1, sizeof(hello1), 500); // 阻塞式发送
  // HAL_Delay(10);
  // uint8_t hello2[] = "Hello, none blocking\n";
  // HAL_UART_Transmit_IT(&huart1, hello2, sizeof(hello2)); // 非阻塞式发送

  // HAL_UART_Receive_IT(&huart1, rxBuffer, RX_CMD_LEN); // 串口不定长数据接收
  HAL_UART_AbortTransmit(&huart1);
  // HAL_UART_AbortTransmit(&huart2);
  /*注意一定要启动定时器中断，使能TIM1-NVIC*/
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim1);
  /*SN74HC595*/
  for (i = 0; i < 6; i++)
    LED6[i] = DIS_BLACK; // 上电消隐
  /*ADC*/
  HAL_ADCEx_Calibration_Start(&hadc1);     // 校准ADC
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn); // DMA1通道1使能
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_value, 5);
  // VREFINT_CAL=Get_VREFINT_CAL();
  HAL_Delay(100); // 等待ADC校准完成
  /*bluetooth*/
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData)); // 串口不定长数据接收
  __HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);                        // 关闭DMA接收过半中断
  // PRINTF_HC05("HC05 init\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (1 == flag_TIM14)
    {
      flag_TIM14 = 0;
      if (++EVENT_700ms >= 70)
      {
        EVENT_700ms = 0;
        SHT40_Read(&humidity, &temperature);
        // TOG_LED_2();
      }
      if (++EVENT_500ms >= 50)
      {
        EVENT_500ms = 0;
        // SMG_Display(display_flag);
        // printf("EVENT_500ms\r\n");
        convertCALvoltage();
        SMG_Display(display_flag);
      }
      if (++EVENT_900ms >= 90)
      {
        EVENT_900ms = 0;
        TOG_LED_1();

        SIG_vol = voltageValue[1] * 100 * 2.0;
        voltagePercent = (voltageValue[1] * 2.0 / 4.2) * 1000;
        PRINTF_PC("Humidity:%.2f %% Temperature:%.2f C ADC3:%.2f  ADC4:%.2f  VCAL:%.2f   DIS:%.2d\r\n", voltageValue[0], voltageValue[1], mcutem, voltageValue[3], voltageValue[4], voltagePercent);
        // PRINTF_HC05("Humidity:%.2f %% Temperature:%.2f C ADC3:%.2f  ADC4:%.2f VCAL:%.2f  DIS:%.2d\r\n", voltageValue[0], voltageValue[1], mcutem, voltageValue[3], voltageValue[4], display_flag);
        // PRINTF_HC05("Humidity:%.2d %% Temperature:%.2d C\r\n", ADC_value[3], ADC_value[4]);
        //  LED6[3] = (uint8_t)(Voltage / 100);
        //  LED6[4] = (uint8_t)((Voltage % 100) / 10);
        //  LED6[4] |= 0x20; // 显示小数位
        //  LED6[5] = (uint8_t)(Voltage % 10);
        //  printf("EVENT_900ms\r\n");
      }
      if (++EVENT_1100ms >= 110)
      {
        EVENT_1100ms = 0;
        EVENT_1100ms_cnt++; // EVENT_1100ms事件计数，超过3次后，再下一次进入TIM14中断，将flag_SLEEP置1，进入SLEEP模式
        // TOG_LED_2();
        // if (voltageValue[0] >= 1.5)
        // {
        //   TOG_LED_2();
        // }
        // HAL_UART_Receive(&huart2, &rx, 1, HAL_MAX_DELAY);
        // if (rx=='1')
        // {
        //   TOG_LED_2();
        //   /* code */
        // }

        /* code */
      }
    }
    if (KEY_EVENT != 0) // 存在按键触发未触发
    {
      if ((KEY_EVENT & 0x01) != 0)
      {
        KEY_EVENT &= ~0x01;  // 清除对应标志位
        display_flag = 0x01; // 显示电池电压
      }
      if ((KEY_EVENT & 0x04) != 0)
      {
        KEY_EVENT &= ~0x04;  // 清除对应标志位
        display_flag = 0x00; // 默认显示温湿度
      }
    }
    if (flag_SLEEP == 1) // 进入SLEEP模式
    {
      HAL_ADC_Stop_DMA(&hadc1);                                         // 手动停止ADC DMA传输
      HAL_TIM_Base_Stop_IT(&htim14);                                    // 关闭TIM14中断
      LED6_RST();                                                       // 优化唤醒显示（SMG全部关闭）
      HAL_Delay(6);                                                     // 留出时间给TIM16更新显示,彻底进入消隐
      HAL_TIM_Base_Stop_IT(&htim16);                                    // 关闭TIM16中断
      LED_Init(0x00);                                                   // 关闭LED
      display_flag = 0x11;                                              // 显示挡位切换到未用挡位上，防止下一次唤醒进入上一次的显示挡位
      HAL_SuspendTick();                                                // 暂停滴答定时器，防止通过滴答定时器中断唤醒
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); // 执行WFI指令,进入SLEEP模式
    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableLSECSS();
}

/* USER CODE BEGIN 4 */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   if (GPIO_Pin == WAKEUP_Pin)
//   {

//     if (GPIO_PIN_SET == KEY_EVENT_WAKE_STATUS()) /* condition */
//     {
//       TOG_LED_2();
//       KEY_EVENT |= 0x00; /* code */
//     }
//   }
//   // if (GPIO_Pin == KEY_Pin)
//   // {
//   //   if (GPIO_PIN_RESET == KEY_EVENT_STATUS()) /* condition */
//   //   {
//   //     KEY_EVENT |= 0x04; /* code */
//   //   }
//   // }
// }

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (flag_SLEEP == 1) // 若按键按下，在睡眠模式，则唤醒。防止未休眠反复唤醒
  {
    flag_SLEEP = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_value, 5);
    HAL_TIM_Base_Start_IT(&htim14);
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_ResumeTick(); // 恢复系统时钟中断
    /*恢复系统时钟中断，MCU才能回到SLEEP前的状态并正常运行*/
  }
  // 每次按键按下，刷新睡眠倒计时，重新开始倒计时
  EVENT_1100ms_cnt = 0;
  if (GPIO_Pin == WAKEUP_Pin)
  {
    if (GPIO_PIN_RESET == KEY_EVENT_WAKE_STATUS()) // 若确实按下，注意外部硬件上拉了
    {
      // TOG_LED_2();
      // KEY_EVENT = 0x04; // KEY执行触发事件，系统未处理可叠加
      switch (currentMode)
      {
      case MODE1:
        currentMode = MODE2;
        KEY_EVENT = 0x01;
        break;
      case MODE2:
        currentMode = MODE1;
        KEY_EVENT = 0x04;
        break;
      }
    }
    // if (GPIO_Pin == KEY_Pin)
    //   {
    //     if (GPIO_PIN_RESET == KEY_EVENT_STATUS()) /* condition */
    //     {
    //       KEY_EVENT |= 0x04; /* code */
    //     }
    //   }
  }
}

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//   if (huart == &huart2)
//   {
//     HAL_UART_Transmit_DMA(&huart2, receiveData, sizeof(receiveData));
//     // if (receiveData[0]==0xAA)//验证数据包第0位是否是我们规定的包头0xAA
//     // {
//     //   TOG_LED_2();
//     //   // if (receiveData[1] == Size)//数据包第一位就是数据包长度
//     //   // {
//     //   //   /*校验和的计算*/
//     //   //   uint8_t checkSum = 0;
//     //   //   for (int i = 0; i < Size-1; i++)
//     //   //   {
//     //   //     checkSum += receiveData[i];
//     //   //   }
//     //   //   if (checkSum==receiveData[Size-1])//校验和是否正确
//     //   //   {
//     //   //     SUM = checkSum;
//     //   //     for (int i = 2; i < Size - 1; i += 2)
//     //   //     {
//     //   //       //解析数据包

//     //   //       if (receiveData[i+1]==0x00)
//     //   //       {
//     //   //         if (receiveData[i] == 0x01) // 蓝色小灯
//     //   //         {
//     //   //           // TOG_LED_2();
//     //   //         }
//     //   //       }
//     //   //     }

//     //   //   }

//     //   // }

//     // }

//     HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData)); // 串口不定长数据接收
//     __HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);                        // 关闭DMA接收过半中断
//   }
// }
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

#ifdef USE_FULL_ASSERT
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
