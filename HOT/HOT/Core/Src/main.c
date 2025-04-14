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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
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
uint8_t wake_pressed = 0; // wake按键状态标志
// 定义光标可选位置的结构体
typedef struct
{
  uint8_t x;      // x坐标
  uint8_t y;      // y坐标
  uint8_t width;  // 区域宽度
  uint8_t height; // 区域高度
} CursorPos_t;

// 定义所有可选位置(根据实际显示位置调整)
const CursorPos_t cursor_positions[] = {
    {20, 24, 24, 8}, // HOT1位置 (4个字符 * 6像素宽度)
    {2, 35, 42, 8},  // HOT1 Target位置 ("Target:" + 2位数字)
    {2, 50, 42, 8},  // HOT1 Reality位置 ("Reality:" + 2位数字)
    {84, 24, 24, 8}, // HOT2位置 (4个字符 * 6像素宽度)
    {65, 35, 42, 8}, // HOT2 Target位置 ("Target:" + 2位数字)
    {65, 50, 42, 8}  // HOT2 Reality位置 ("Reality:" + 2位数字)
};

#define CURSOR_POS_NUM (sizeof(cursor_positions) / sizeof(cursor_positions[0]))
uint8_t current_cursor = 0; // 当前光标位置索引

uint8_t key1_pressed = 0;       // KEY1按键状态标志
uint8_t key2_pressed = 0;       // KEY2按键状态标志
uint32_t key1_press_time = 0;   // KEY1按下时间
uint32_t key2_press_time = 0;   // KEY2按下时间
static uint8_t key1_filter = 0; // KEY1按键滤波器
static uint8_t key2_filter = 0; // KEY2按键滤波器
static uint8_t wake_filter = 0; // WAKE按键滤波器
#define KEY_FILTER_TIME 10      // 按键滤波时间5ms

uint8_t menu_level = 0;  // 0:主菜单(HOT1/2选择) 1:子菜单(Target/Reality选择)
uint8_t current_hot = 0; // 0:HOT1子菜单 1:HOT2子菜单

// 添加定时计数变量
static uint16_t wake_long_press_cnt = 0; // WAKE按键长按计数器
#define LONG_PRESS_CNT 3000              // 长按时间计数阈值(5秒,定时器1ms中断)
uint8_t menu_switching = 0;              // 添加菜单切换标志，防止重复触发

// 添加参数编辑相关变量
uint8_t param_editing = 0; // 参数编辑状态标志
uint8_t blink_state = 0;   // 闪烁状态
uint16_t blink_cnt = 0;    // 闪烁计数器
#define BLINK_PERIOD 500   // 闪烁周期(ms)

// 添加温度参数变量
uint16_t hot1_target = 30; // HOT1目标温度
float hot1_reality = 0.0;  // HOT1实际温度
uint16_t hot2_target = 30; // HOT2目标温度
float hot2_reality = 0.0;  // HOT2实际温度

#define FLASH_TARGET_ADDR 0x0800F800                              // 使用倒数第二页存储参数(31页,距离末尾4KB)
#define MY_FLASH_PAGE_SIZE 0x800                                  // G030的页大小为2KB (改名避免冲突)
#define Target1_Temperture (*(uint32_t *)FLASH_TARGET_ADDR)       // HOT1目标温度存储地址
#define Target2_Temperture (*(uint32_t *)(FLASH_TARGET_ADDR + 4)) // HOT2目标温度存储地址

// ADC采样相关定义
#define ADC_CHANNEL_NUM 8            // 采样通道数
#define ADC_SAMPLE_NUM 10            // 每个通道采样次数
uint16_t adc_buf[ADC_CHANNEL_NUM];   // DMA缓存区
uint16_t adc_value[ADC_CHANNEL_NUM]; // 各通道平均值

// 添加滤波相关定义
#define N 10 // 滑动平均滤波点数
#define M 5  // 限幅平均滤波点数
#define A 50 // 限幅值

// 滤波缓存数组
static uint16_t value_buf[ADC_CHANNEL_NUM][N];    // 滑动平均滤波缓存
static uint16_t filter_data[ADC_CHANNEL_NUM][M];  // 限幅平均滤波缓存
static uint8_t filter_cnt[ADC_CHANNEL_NUM] = {0}; // 滤波计数器

// 添加ADC定时采样相关变量
static uint16_t adc_sample_cnt = 0; // ADC采样计数器
#define ADC_SAMPLE_PERIOD 30        // ADC采样周期(30ms)

float NTC1_R = 0.0f;
float NTC2_R = 0.0f;

// 在文件开头的变量声明部分添加OLED刷新计数器
static uint16_t oled_refresh_cnt = 0; // OLED刷新计数器
#define OLED_REFRESH_PERIOD 500       // OLED刷新周期(500ms)

// 串口输出
char message[50] = ""; // 用于存储输出信息的缓冲区
/*参考电压值*/
float Vref = 0.0f;    // 参考电压值
float Voltage = 0.0f; // 电压值
/*打印控制*/
static uint32_t print_tick = 0;
#define PRINT_PERIOD 1000 // 打印周期1000ms

/*PID定义*/
typedef struct
{
  float desired;           // 期望值
  float current;           // 实际值
  float error;             // 误差值
  float error_last;        // 上次误差值
  float error_last_last;   // 上上次误差值
  float incremental_value; // 增量值
  float Kp;                // 比例系数
  float Ki;                // 积分系数
  float Kd;                // 微分系数
  float out;               // 输出值
} PID_Object;

PID_Object PID_Ctrl1; // PID对象
PID_Object PID_Ctrl2; // PID对象

#define SOFT_START_PERIOD 600 // 软启动时间(ms)
#define SOFT_START_MAX 600    // 软启动最大值
uint16_t PWM1 = 0;
uint16_t PWM2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void OLED_DrawInterface(void)
{
  uint8_t i;

  // 1. 清屏并初始化
  OLED_Clear();
  OLED_Update();

  // 2. 显示标题 "HOT" 在顶部中间
  OLED_ShowString(52, 0, "HOT", OLED_8X16);
  OLED_Update();

  // 3. 加粗的虚线分隔 (每2像素画一个点)
  for (i = 0; i < 128; i += 2)
  {
    for (uint8_t j = 0; j < 2; j++)
    {
      OLED_DrawPoint(i, 16 + j);
    }
  }
  OLED_Update();

  // 4. 加粗边框线和分隔线
  for (i = 16; i < 64; i++)
  {
    // 左边框加粗
    OLED_DrawPoint(0, i);
    OLED_DrawPoint(1, i);

    // 中间分隔线加粗
    OLED_DrawPoint(63, i);
    OLED_DrawPoint(64, i);

    // // 右边框加粗
    // OLED_DrawPoint(126, i);
    // OLED_DrawPoint(127, i);
  }
  OLED_Update();

  // 5. 左侧HOT1列 (稍微向右移动以避免边框)
  OLED_ShowString(20, 24, "HOT1", OLED_6X8);
  OLED_ShowString(2, 35, "Target:", OLED_6X8);
  OLED_ShowString(2, 50, "Reality:", OLED_6X8);
  OLED_ShowNum(50, 35, hot1_target, 2, OLED_6X8);
  OLED_ShowNum(50, 50, hot1_reality, 2, OLED_6X8);
  OLED_Update();

  // 6. 右侧HOT2列 (稍微向左移动以避免边框)
  OLED_ShowString(84, 24, "HOT2", OLED_6X8);
  OLED_ShowString(65, 35, "Target:", OLED_6X8);
  OLED_ShowString(65, 50, "Reality:", OLED_6X8);
  OLED_ShowNum(112, 35, hot2_target, 2, OLED_6X8);
  OLED_ShowNum(112, 50, hot2_reality, 2, OLED_6X8);
  OLED_Update();
}

/* 添加光标显示函数 */
void ShowCursor(uint8_t position_index)
{
  static uint8_t last_position = 0xFF; // 初始化为无效值

  // 如果有上一个位置，先恢复原来的显示
  if (last_position != 0xFF)
  {
    OLED_ReverseArea(cursor_positions[last_position].x,
                     cursor_positions[last_position].y,
                     cursor_positions[last_position].width,
                     cursor_positions[last_position].height);
  }

  // 显示新位置的光标
  OLED_ReverseArea(cursor_positions[position_index].x,
                   cursor_positions[position_index].y,
                   cursor_positions[position_index].width,
                   cursor_positions[position_index].height);

  last_position = position_index;
  OLED_Update();
}

// Flash写入函数
HAL_StatusTypeDef Flash_Write_Data(uint32_t Address, uint32_t *Data, uint16_t DataSize)
{
  HAL_StatusTypeDef status = HAL_OK;

  // 解锁Flash
  HAL_FLASH_Unlock();

  // 擦除倒数第二页
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = 31; // G030使用倒数第二页(总共32页,0-31)
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)
  {
    // 写入数据
    for (uint16_t i = 0; i < DataSize; i++)
    {
      uint64_t data64 = ((uint64_t)Data[i] << 32) | 0x55AA55AA; // 添加校验标记
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            Address + (i * 8),
                            data64) != HAL_OK)
      {
        status = HAL_ERROR;
        break;
      }
    }
  }
  else
  {
    status = HAL_ERROR;
  }

  // 锁定Flash
  HAL_FLASH_Lock();

  return status;
}

/*计算出当前NTC热敏电阻的阻值*/
float ADC_GetResistance(uint32_t adc_value)
{
  return (adc_value / (4095.0f - adc_value)) * 10000.0f;
}

/*根据方程计算出当前的温度值*/
float Resistance2Temerature(uint16_t Rsistance)
{
  float B = 3950.0f;   // B值
  float R0 = 10000.0f; // 25度时的电阻值
  float T0 = 25.0f;
  return (1.0f / ((1.0f / B) * log(Rsistance / R0) + 1.0f / (T0 + 273.15f)) - 273.15f);
}

// 滑动平均滤波函数
uint16_t moveAverageFilter(uint8_t channel)
{
  static uint32_t sum[ADC_CHANNEL_NUM] = {0};
  static uint8_t cnt[ADC_CHANNEL_NUM] = {0};

  // 使用当前ADC值
  uint16_t current_value = adc_buf[channel];

  if (cnt[channel] < N) // 缓冲区未满
  {
    value_buf[channel][cnt[channel]] = current_value;
    sum[channel] += current_value;
    cnt[channel]++;

    // 初始阶段直接返回当前值，避免被0值影响
    return current_value;
    // 或者返回平均值：return sum[channel] / cnt[channel];
  }
  else // 缓冲区已满
  {
    sum[channel] -= value_buf[channel][0];
    // 移动数据
    for (uint8_t i = 0; i < N - 1; i++)
    {
      value_buf[channel][i] = value_buf[channel][i + 1];
    }
    value_buf[channel][N - 1] = current_value;
    sum[channel] += current_value;
    return sum[channel] / N;
  }
}

// 限幅平均滤波函数
uint16_t LAverageFilter(uint8_t channel)
{
  static uint8_t initialized = 0;
  uint16_t filtered = moveAverageFilter(channel);

  // 初始化阶段直接返回当前值
  if (!initialized)
  {
    for (uint8_t i = 0; i < M; i++)
    {
      filter_data[channel][i] = filtered;
    }
    filter_cnt[channel] = M;
    initialized = 1;
    return filtered;
  }

  if (filter_cnt[channel] > 0)
  {
    // 检查限幅 - 增大限幅值
    if (abs(filtered - filter_data[channel][filter_cnt[channel] - 1]) > 50) // 增大限幅值
    {
      return filter_data[channel][filter_cnt[channel] - 1];
    }
  }

  // 移动数据窗口
  if (filter_cnt[channel] < M)
  {
    filter_data[channel][filter_cnt[channel]] = filtered;
    filter_cnt[channel]++;
  }
  else
  {
    for (uint8_t i = 0; i < M - 1; i++)
    {
      filter_data[channel][i] = filter_data[channel][i + 1];
    }
    filter_data[channel][M - 1] = filtered;
  }

  // 计算平均值
  uint32_t sum = 0;
  for (uint8_t i = 0; i < M; i++)
  {
    sum += filter_data[channel][i];
  }
  return sum / M;
}

void PID_INIT(PID_Object *pid, float kp, float ki, float kd, float initial_out)
{
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;

  pid->desired = 0.0f;
  pid->current = 0.0f;
  pid->error = pid->error_last = pid->error_last_last = 0.0f;

  pid->out = initial_out;
}

/*PID核心算法*/
float PID_calculate(PID_Object *pid, float desired, float current)
{
  pid->error = desired - current;

  pid->incremental_value = (pid->Kp * (pid->error - pid->error_last) + pid->Ki * pid->error + pid->Kd * (pid->error - 2 * pid->error_last + pid->error_last_last));
  pid->out += pid->incremental_value;

  pid->error_last = pid->error;
  pid->error_last_last = pid->error_last;

  if (pid->out >= 800)
  {
    pid->out = 800;
  } // 限制最大最小占空比
  if (pid->out <= 10)
  {
    pid->out = 10;
  }
  return pid->out;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 启动TIM3中断
  HAL_TIM_Base_Start_IT(&htim3);
  // 启动TIM1 PWM输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 启动TIM1通道1的PWM输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 启动TIM1通道2的PWM输出
  // 从Flash读取温度参数
  uint64_t temp1 = *(uint64_t *)FLASH_TARGET_ADDR;
  uint64_t temp2 = *(uint64_t *)(FLASH_TARGET_ADDR + 8);

  // 检查校验标记并读取数据
  if ((temp1 & 0xFFFFFFFF) == 0x55AA55AA)
  {
    hot1_target = (temp1 >> 32) & 0xFF;
  }

  if ((temp2 & 0xFFFFFFFF) == 0x55AA55AA)
  {
    hot2_target = (temp2 >> 32) & 0xFF;
  }
  // 初始化OLED
  OLED_Init();
  // 清屏
  OLED_Clear();
  OLED_DrawInterface();
  HAL_Delay(10); // 等待界面显示稳定
  ShowCursor(0); // 显示初始光标位置

  // 初始化滤波缓冲区
  for (uint8_t ch = 0; ch < ADC_CHANNEL_NUM; ch++)
  {
    for (uint8_t i = 0; i < N; i++)
    {
      value_buf[ch][i] = 0;
    }
    for (uint8_t i = 0; i < M; i++)
    {
      filter_data[ch][i] = 0;
    }
    filter_cnt[ch] = 0;
  }

  // 启动ADC和DMA采样
  HAL_ADCEx_Calibration_Start(&hadc1); // ADC校准
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNEL_NUM);

  PID_INIT(&PID_Ctrl1, 300.0f, 70.0f, 0.0f, 100.0f); // 第一个PID
  PID_INIT(&PID_Ctrl2, 250.0f, 50.0f, 0.0f, 100.0f); // 第二个PID

  // 软启动过程
   for (uint16_t i = 0; i < SOFT_START_MAX; i++)
   {
     PWM1 = i;
     PWM2 = i;
     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWM2);
     HAL_Delay(1);
	 }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // WAKE按键处理
    if (HAL_GPIO_ReadPin(WAKE_GPIO_Port, WAKE_Pin) == GPIO_PIN_RESET)
    {
      wake_filter++;
      if (wake_filter >= KEY_FILTER_TIME && !wake_pressed)
      {
        wake_pressed = 1;
        wake_long_press_cnt = 0;

        // 判断是否在参数编辑模式
        if (param_editing)
        {
          // 退出参数编辑模式
          param_editing = 0;

          // 准备保存的数据
          uint32_t flash_data[2];
          flash_data[0] = hot1_target;
          flash_data[1] = hot2_target;

          // 保存到Flash
          if (Flash_Write_Data(FLASH_TARGET_ADDR, flash_data, 2) != HAL_OK)
          {
            Error_Handler(); // Flash写入错误处理
          }

          // 更新显示
          if (current_hot == 0)
          {
            OLED_ShowNum(50, 35, hot1_target, 2, OLED_6X8);
          }
          else
          {
            OLED_ShowNum(112, 35, hot2_target, 2, OLED_6X8);
          }
          OLED_Update();
        }
        // 不在参数编辑模式时的菜单切换
        else if (menu_level == 0 && !menu_switching)
        {
          // 从主菜单进入子菜单
          menu_level = 1;
          current_hot = (current_cursor == 0) ? 0 : 1;
          current_cursor = (current_hot == 0) ? 1 : 4;
          ShowCursor(current_cursor);
        }
        else if (menu_level == 1 && !menu_switching && !param_editing)
        {
          // 在子菜单中且不在编辑状态时，处理Target编辑
          if (current_cursor == 1 || current_cursor == 4) // Target位置
          {
            param_editing = 1;
            blink_state = 1;
            blink_cnt = 0;
          }
        }
      }

      // 按键持续按下时的长按处理
      if (wake_pressed)
      {
        wake_long_press_cnt++;
        if (wake_long_press_cnt >= LONG_PRESS_CNT) // 达到长按时间阈值(5秒)
        {
          if (menu_level == 1 && !menu_switching) // 在子菜单中
          {
            // 返回主菜单
            menu_level = 0;
            current_cursor = current_hot ? 3 : 0;
            ShowCursor(current_cursor);
            menu_switching = 1;
            param_editing = 0;
          }
          wake_long_press_cnt = LONG_PRESS_CNT; // 防止计数器溢出
        }
      }
    }
    else
    {
      wake_pressed = 0;
      wake_filter = 0;
      wake_long_press_cnt = 0;
      menu_switching = 0;
    }

    // LED闪烁处理
    static uint16_t led_cnt = 0;
    led_cnt++;
    if (led_cnt >= 500) // 500ms翻转一次
    {
      led_cnt = 0;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }

    // 参数编辑模式下的闪烁处理
    if (param_editing)
    {
      blink_cnt++;
      if (blink_cnt >= BLINK_PERIOD)
      {
        blink_cnt = 0;
        blink_state = !blink_state;

        // 更新显示
        if (current_hot == 0)
        {
          if (blink_state)
            OLED_ShowNum(50, 35, hot1_target, 2, OLED_6X8);
          else
            OLED_ShowString(50, 35, "  ", OLED_6X8);
        }
        else
        {
          if (blink_state)
            OLED_ShowNum(112, 35, hot2_target, 2, OLED_6X8);
          else
            OLED_ShowString(112, 35, "  ", OLED_6X8);
        }
        OLED_Update();
      }
    }

    // KEY1按键处理(上移/循环)
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      key1_filter++;
      if (key1_filter >= KEY_FILTER_TIME && !key1_pressed)
      {
        key1_pressed = 1;
        if (param_editing)
        {
          // 参数编辑模式下增加温度值
          if (current_hot == 0)
          {
            if (hot1_target < 99)
              hot1_target++;
            OLED_ShowNum(50, 35, hot1_target, 2, OLED_6X8);
          }
          else
          {
            if (hot2_target < 99)
              hot2_target++;
            OLED_ShowNum(112, 35, hot2_target, 2, OLED_6X8);
          }
          OLED_Update();
          blink_state = 1;
          blink_cnt = 0;
        }
        else if (menu_level == 0)
        {
          // 主菜单中HOT1和HOT2循环切换
          current_cursor = (current_cursor == 0) ? 3 : 0; // 在0和3之间循环
          ShowCursor(current_cursor);
        }
        else
        {
          // 子菜单中在Target和Reality之间循环切换
          if (current_hot == 0) // HOT1子菜单
          {
            current_cursor = (current_cursor == 1) ? 2 : 1; // 在1和2之间循环
          }
          else // HOT2子菜单
          {
            current_cursor = (current_cursor == 4) ? 5 : 4; // 在4和5之间循环
          }
          ShowCursor(current_cursor);
        }
      }
    }
    else
    {
      key1_pressed = 0;
      key1_filter = 0;
    }

    // KEY2按键处理(下移/循环)
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
      key2_filter++;
      if (key2_filter >= KEY_FILTER_TIME && !key2_pressed)
      {
        key2_pressed = 1;
        if (param_editing)
        {
          // 参数编辑模式下减少温度值
          if (current_hot == 0)
          {
            if (hot1_target > 0)
              hot1_target--;
            OLED_ShowNum(50, 35, hot1_target, 2, OLED_6X8);
          }
          else
          {
            if (hot2_target > 0)
              hot2_target--;
            OLED_ShowNum(112, 35, hot2_target, 2, OLED_6X8);
          }
          OLED_Update();
          blink_state = 1;
          blink_cnt = 0;
        }
        else if (menu_level == 0)
        {
          // 主菜单中HOT1和HOT2循环切换
          current_cursor = (current_cursor == 0) ? 3 : 0; // 在0和3之间循环
          ShowCursor(current_cursor);
        }
        else
        {
          // 子菜单中在Target和Reality之间循环切换
          if (current_hot == 0) // HOT1子菜单
          {
            current_cursor = (current_cursor == 1) ? 2 : 1; // 在1和2之间循环
          }
          else // HOT2子菜单
          {
            current_cursor = (current_cursor == 4) ? 5 : 4; // 在4和5之间循环
          }
          ShowCursor(current_cursor);
        }
      }
    }
    else
    {
      key2_pressed = 0;
      key2_filter = 0;
    }

    adc_sample_cnt++;
    if (adc_sample_cnt >= ADC_SAMPLE_PERIOD)
    {
      adc_sample_cnt = 0;

      // 保存并打印原始ADC值
      for (uint8_t i = 0; i < ADC_CHANNEL_NUM; i++)
      {
        adc_value[i] = adc_buf[i];
      }

      // 打印滤波后的值
      uint16_t filtered_adc4 = LAverageFilter(4);
      uint16_t filtered_adc5 = LAverageFilter(5);

      // 打印计算的电阻值
      NTC1_R = ADC_GetResistance(filtered_adc4);
      NTC2_R = ADC_GetResistance(filtered_adc5);

      // 计算温度值
      hot1_reality = Resistance2Temerature(NTC1_R);
      hot2_reality = Resistance2Temerature(NTC2_R);

      // // 更新PID控制
       PWM1 = (uint16_t)PID_calculate(&PID_Ctrl1, hot1_target, hot1_reality);
//       PWM2 = (uint16_t)PID_calculate(&PID_Ctrl2, hot2_target, hot2_reality);

      // // 更新PWM输出
       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM1);
//       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWM2);
    }
    // OLED刷新控制
    oled_refresh_cnt++;
    if (oled_refresh_cnt >= OLED_REFRESH_PERIOD)
    {
      oled_refresh_cnt = 0;

      // 更新OLED显示的实际温度值
      if (!param_editing) // 如果不在参数编辑模式下
      {
        // 更新HOT1的实际温度显示
        OLED_ShowNum(50, 50, hot1_reality, 2, OLED_6X8);
        // 更新HOT2的实际温度显示
        OLED_ShowNum(112, 50, hot2_reality, 2, OLED_6X8);
        // 更新显示到OLED屏幕
        OLED_Update();
      }
    }
    // 串口打印控制
    print_tick++;
    if (print_tick >= 33) // 约1秒打印一次
    {
      print_tick = 0; // 重置打印计数器
      sprintf(message, "Temperature1: %.2f,%d,%.2f,%d\r\n", hot1_reality, hot1_target, hot2_reality, hot2_target);
      HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); // 发送数据到串口
      // sprintf(message, "Reality Temperature2: %.2f\r\n", hot2_reality);
      // HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); // 发送数据到串口
      // sprintf(message, "Target Temperature1: %d\r\n", hot1_target);
      // HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
      // sprintf(message, "Target Temperature2: %d\r\n", hot2_target);
      // HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); // 发送数据到串口
    }
  }
}

// // DMA完成回调函数中处理ADC数据
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//   if (hadc->Instance == ADC1)
//   {
//     sprintf(message, "ADC1 DMA complete: %d %d %d %d %d %d %d %d\r\n",
//             adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4], adc_buf[5], adc_buf[6], adc_buf[7]);
//     HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); // 发送数据到串口
//   }
// }
/*启动了DMA连续模式就不用使用中断回调了*/

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
