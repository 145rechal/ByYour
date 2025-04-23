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

// 修改光标位置定义，使宽度与文本匹配
CursorPos_t cursor_positions[] = {
    {40, 35, 36, 8}, // Target位置 - 宽度36对应6个6×8字体
    {40, 50, 42, 8}, // Reality位置 - 宽度42对应7个6×8字体
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
#define LONG_PRESS_CNT 1800              // 长按时间计数阈值(5秒,定时器1ms中断)
uint8_t menu_switching = 0;              // 添加菜单切换标志，防止重复触发
// 添加长按检测变量
static uint16_t key1_long_press_cnt = 0; // KEY1长按计数器
static uint16_t key2_long_press_cnt = 0; // KEY2长按计数器
#define KEY_LONG_PRESS_TIME 300          // 长按检测时间阈值(约300ms)
#define KEY_LONG_PRESS_REPEAT 50         // 长按重复触发时间(约50ms)
static uint16_t key1_repeat_cnt = 0;     // KEY1重复计数器
static uint16_t key2_repeat_cnt = 0;     // KEY2重复计数器

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

// 添加滚动文本变量
#define SCROLL_TEXT_MAX_LENGTH 50                                   // 最大滚动文本长度
char scroll_text[SCROLL_TEXT_MAX_LENGTH] = "Created by Rachel_123"; // 滚动文本内容
int16_t scroll_position = 12;                                       // 初始滚动位置
uint16_t scroll_delay_cnt = 0;                                      // 滚动延迟计数器
#define SCROLL_DELAY 50                                             // 滚动间隔时间(ms)

// 添加温度曲线绘制相关变量
#define CURVE_POINTS 64             // 曲线上的点数
#define CURVE_UPDATE_PERIOD 20      // 曲线更新周期(20ms)
float temp_history[CURVE_POINTS];   // 实际温度历史数据
float target_history[CURVE_POINTS]; // 目标温度历史数据
uint8_t curve_index = 0;            // 当前插入点的索引
uint16_t curve_update_cnt = 0;      // 曲线更新计数器
uint8_t is_reality_page = 0;        // 是否在reality页面标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void OLED_ShowFirstScreen(void)
{
  OLED_Clear();

  // 居中显示标题 "HOT"
  uint8_t title_x = (128 - 8 * 3) / 2;
  OLED_ShowString(title_x, 0, "HOT", OLED_8X16);

  // 初始显示作者信息（后续会滚动）
  OLED_ShowString(scroll_position, 16, scroll_text, OLED_6X8);

  // 绘制分隔线 - 这行很重要，确保它不被注释掉
  for (uint8_t i = 12; i < 116; i++)
  {
    OLED_DrawPoint(i, 25);
  }

  // 计算Target和Reality的居中位置
  uint8_t content_x = 40;

  // 显示Target和Reality（居中）
  OLED_ShowString(content_x, 35, "Target", OLED_6X8);
  OLED_ShowString(content_x, 50, "Reality", OLED_6X8);

  OLED_Update();
}

void ShowCursor(uint8_t position_index)
{
  static uint8_t last_position = 0xFF; // 初始化为无效值

  // 防止索引越界
  if (position_index >= CURSOR_POS_NUM)
  {
    return;
  }

  // 如果位置发生变化，先清除旧光标所在区域
  if (last_position != position_index && last_position < CURSOR_POS_NUM)
  {
    // 恢复旧位置的正常显示
    OLED_ClearArea(cursor_positions[last_position].x,
                   cursor_positions[last_position].y,
                   cursor_positions[last_position].width,
                   cursor_positions[last_position].height);

    // 重新绘制该区域的内容
    uint8_t content_x = 40;
    if (last_position == 0)
      OLED_ShowString(content_x, 35, "Target", OLED_6X8);
    else if (last_position == 1)
      OLED_ShowString(content_x, 50, "Reality", OLED_6X8);
  }

  // 显示新位置的光标
  OLED_ReverseArea(cursor_positions[position_index].x,
                   cursor_positions[position_index].y,
                   cursor_positions[position_index].width,
                   cursor_positions[position_index].height);

  last_position = position_index;

  // 更新显示
  OLED_Update();
}

void OLED_DrawHeatingIcon(uint8_t x, uint8_t y, uint8_t is_heating)
{
  if (is_heating)
  {
    // 绘制加热状态图标（小火焰）
    OLED_DrawPoint(x + 2, y);
    OLED_DrawPoint(x + 3, y);
    OLED_DrawPoint(x + 1, y + 1);
    OLED_DrawPoint(x + 2, y + 1);
    OLED_DrawPoint(x + 3, y + 1);
    OLED_DrawPoint(x + 4, y + 1);
    OLED_DrawPoint(x, y + 2);
    OLED_DrawPoint(x + 1, y + 2);
    OLED_DrawPoint(x + 2, y + 2);
    OLED_DrawPoint(x + 3, y + 2);
    OLED_DrawPoint(x + 4, y + 2);
    OLED_DrawPoint(x + 5, y + 2);
    OLED_DrawPoint(x + 2, y + 3);
    OLED_DrawPoint(x + 3, y + 3);
  }
  else
  {
    // 清除图标区域
    for (uint8_t i = 0; i < 6; i++)
    {
      for (uint8_t j = 0; j < 4; j++)
      {
        OLED_ClearPoint(x + i, y + j);
      }
    }
  }
  OLED_Update();
}
void OLED_UpdateScrollText(void)
{
  // 清除整个滚动文本区域，避免拖影
  OLED_ClearArea(0, 16, 128, 8);

  // 显示滚动文本
  OLED_ShowString(scroll_position, 16, scroll_text, OLED_6X8);

  // 更新滚动位置
  scroll_position--;

  // 如果文本滚动出屏幕，重新从右侧开始
  int threshold = -(strlen(scroll_text) * 6);
  if (scroll_position < threshold)
  {
    scroll_position = 128;
  }

  // 仅更新滚动文本区域
  OLED_UpdateArea(0, 16, 128, 8);

  // 降低光标更新频率，减少屏幕闪烁
  static uint8_t cursor_update_count = 0;
  cursor_update_count++;
  if (cursor_update_count >= 10) // 增加到10，减少更新频率
  {
    cursor_update_count = 0;
    if (menu_level == 0) // 只在主菜单时更新光标
    {
      ShowCursor(current_cursor);
    }
  }
}

// 绘制垂直圆弧数字选择器
void OLED_ShowNumberSelector(uint16_t current_value)
{
  // 清除显示区域 - 垂直区域
  OLED_ClearArea(50, 10, 78, 50);

  // 计算要显示的5个值（当前值和其前后各两个值）
  uint16_t values[5];
  for (int i = 0; i < 5; i++)
  {
    values[i] = current_value - 2 + i;
  }

  // 垂直圆弧的布局 - 纵向排列
  uint8_t y_positions[5] = {15, 25, 35, 45, 55}; // 垂直排列，从上到下

  // 水平位置形成弧度 - 改为朝向Target文本方向的弧形
  uint8_t x_positions[5] = {75, 70, 65, 70, 75}; // 形成朝左的垂直圆弧效果

  // 显示所有数字
  for (int i = 0; i < 5; i++)
  {
    // 中间值用反相显示
    if (i == 2)
    {
      // 先绘制背景
      OLED_FillRect(x_positions[i] - 2, y_positions[i] - 1, 18, 10);
      // 用清除点的方式绘制数字（形成反相效果）
      char num_str[3];
      sprintf(num_str, "%d", values[i]);
      for (int j = 0; j < strlen(num_str); j++)
      {
        OLED_ShowChar(x_positions[i] + j * 6, y_positions[i], num_str[j], OLED_6X8);
      }
    }
    else
    {
      // 普通显示其他数字
      OLED_ShowNum(x_positions[i], y_positions[i], values[i], 2, OLED_6X8);
    }
  }

  // 左侧显示"Target"
  OLED_ShowString(15, 35, "Target:", OLED_6X8);

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

// 绘制温度曲线函数
void OLED_DrawTempCurve(void)
{
  // 清除曲线区域
  OLED_ClearArea(0, 18, 128, 46);

  // 绘制坐标轴
  OLED_DrawLine(0, 63, 127, 63); // X轴
  OLED_DrawLine(0, 18, 0, 63);   // Y轴

  // 绘制刻度
  for (int i = 0; i < 5; i++)
  {
    uint8_t y = 63 - i * 10;
    OLED_DrawLine(0, y, 3, y); // Y轴刻度
  }

  // 计算温度映射的参数
  float min_temp = 15.0f;                      // 最低显示温度
  float max_temp = 85.0f;                      // 最高显示温度
  float scale = 45.0f / (max_temp - min_temp); // 计算温度到像素的缩放比例

  // 绘制实际温度曲线
  for (int i = 0; i < CURVE_POINTS - 1; i++)
  {
    int x1 = i * 2;
    int x2 = (i + 1) * 2;

    // 计算y坐标（OLED屏幕原点在左上角，所以需要反向计算）
    int y1 = 63 - (int)((temp_history[i] - min_temp) * scale);
    int y2 = 63 - (int)((temp_history[i + 1] - min_temp) * scale);

    // 限制y坐标范围
    if (y1 < 18)
      y1 = 18;
    if (y1 > 63)
      y1 = 63;
    if (y2 < 18)
      y2 = 18;
    if (y2 > 63)
      y2 = 63;

    // 绘制线段
    OLED_DrawLine(x1, y1, x2, y2);
  }

  // 绘制目标温度曲线（使用虚线或不同模式）
  for (int i = 0; i < CURVE_POINTS - 1; i++)
  {
    int x1 = i * 2;
    int x2 = (i + 1) * 2;

    // 计算y坐标
    int y1 = 63 - (int)((target_history[i] - min_temp) * scale);
    int y2 = 63 - (int)((target_history[i + 1] - min_temp) * scale);

    // 限制y坐标范围
    if (y1 < 18)
      y1 = 18;
    if (y1 > 63)
      y1 = 63;
    if (y2 < 18)
      y2 = 18;
    if (y2 > 63)
      y2 = 63;

    // 绘制虚线（每隔一定距离绘制点）
    if (i % 2 == 0)
    {
      OLED_DrawPoint(x1, y1);
    }
  }

  // 显示PWM值
  OLED_ShowString(5, 5, "PWM:", OLED_6X8);
  OLED_ShowNum(35, 5, PWM1, 3, OLED_6X8);

  // 显示图例
  OLED_ShowString(70, 5, "T:", OLED_6X8);
  OLED_ShowNum(82, 5, (uint16_t)hot1_reality, 2, OLED_6X8);

  // 更新显示
  OLED_Update();
}

/*计算出当前NTC热敏电阻的阻值*/
float ADC_GetResistance(uint32_t adc_value)
{
  return (adc_value / (4095.0f - adc_value)) * 10000.0f;
}

/*根据方程计算出当前的温度值*/
float Resistance2Temerature(float Rsistance)
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

  if (pid->out >= 750)
  {
    pid->out = 750;
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
  // 显示第一个界面
  OLED_ShowFirstScreen();
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

  PID_INIT(&PID_Ctrl1, 300.0f, 70.0f, 0.0f, 100.0f); // PID

  // 软启动过程
  for (uint16_t i = 0; i < SOFT_START_MAX; i++)
  {
    PWM1 = i;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM1);
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

        // 在主菜单状态下（初始页面）
        if (menu_level == 0 && !menu_switching)
        {
          menu_level = 1; // 进入子菜单

          // 进入Target页面
          if (current_cursor == 0) // 当前在Target位置
          {
            // 进入Target页面
            OLED_Clear();
            // 使用数字选择器显示目标温度
            OLED_ShowNumberSelector(hot1_target);
            OLED_Update();
          }
          else if (current_cursor == 1) // 当前在Reality位置
          {
            // 进入Reality页面
            OLED_Clear();
            // 初始化温度历史数据
            is_reality_page = 1; // 设置在reality页面标志

            // 初始化历史数据数组
            for (int i = 0; i < CURVE_POINTS; i++)
            {
              temp_history[i] = hot1_reality;
              target_history[i] = hot1_target;
            }

            // 绘制初始曲线
            OLED_DrawTempCurve();

            menu_switching = 1; // 防止重复触发
          }

          menu_switching = 1; // 防止重复触发
        }
        // 在Target页面下短按wake就保存温度并返回主菜单
        else if (menu_level == 1 && !menu_switching && current_cursor == 0)
        {
          // 保存参数到Flash
          uint32_t flash_data[2];
          flash_data[0] = hot1_target;
          flash_data[1] = hot2_target;

          if (Flash_Write_Data(FLASH_TARGET_ADDR, flash_data, 2) != HAL_OK)
          {
            Error_Handler();
          }

          // 返回主菜单
          menu_level = 0;

          // 重置滚动文本位置，确保返回主菜单时从初始位置开始
          scroll_position = 12;

          // 重新显示初始界面
          OLED_ShowFirstScreen();
          ShowCursor(current_cursor); // 还原原来的光标位置

          menu_switching = 1; // 防止重复触发
        }
        // Reality页面处理保持不变
      }

      // 长按处理 - 只保留从Reality页面返回的功能
      if (wake_pressed)
      {
        wake_long_press_cnt++;
        if (wake_long_press_cnt >= LONG_PRESS_CNT) // 达到长按阈值(3秒)
        {
          if (menu_level == 1 && !menu_switching && current_cursor == 1) // 在Reality子菜单中
          {
            // 返回主菜单
            menu_level = 0;
            is_reality_page = 0; // 清除在reality页面标志

            // 重置滚动文本位置，确保返回主菜单时从初始位置开始
            scroll_position = 12;

            // 重新显示初始界面
            OLED_ShowFirstScreen();
            ShowCursor(current_cursor); // 还原原来的光标位置

            menu_switching = 1; // 防止重复触发
          }
          wake_long_press_cnt = LONG_PRESS_CNT; // 防止计数器溢出
        }
      }
    }
    else
    {
      // 按键松开处理保持不变
      wake_pressed = 0;
      wake_filter = 0;
      wake_long_press_cnt = 0;
      menu_switching = 0; // 清除菜单切换标志
    }

    // LED闪烁处理
    static uint16_t led_cnt = 0;
    led_cnt++;
    if (led_cnt >= 500) // 500ms翻转一次
    {
      led_cnt = 0;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }

    // 修改滚动文本更新处理，只在主菜单时更新滚动文本
    scroll_delay_cnt++;
    if (scroll_delay_cnt >= SCROLL_DELAY)
    {
      scroll_delay_cnt = 0;
      // 只在主菜单(menu_level==0)时更新滚动文本
      if (menu_level == 0)
      {
        OLED_UpdateScrollText(); // 调用滚动更新函数
      }
    }

    // 修改KEY1按键处理，添加长按功能
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      key1_filter++;
      if (key1_filter >= KEY_FILTER_TIME && !key1_pressed)
      {
        key1_pressed = 1;
        key1_long_press_cnt = 0;
        key1_repeat_cnt = 0;

        // 短按操作处理（保持原有逻辑）
        // 在主菜单时进行Target和Reality之间的切换
        if (menu_level == 0)
        {
          // 限制在Target和Reality之间切换
          if (current_cursor <= 1) // 确保光标在有效范围内
          {
            current_cursor = (current_cursor == 0) ? 1 : 0;
            ShowCursor(current_cursor);
          }
        }
        // 在Target页面时，直接增加温度值
        else if (menu_level == 1 && current_cursor == 0)
        {
          if (hot1_target < 80)
            hot1_target++;                      // 限制最大温度
          OLED_ShowNumberSelector(hot1_target); // 更新数字选择器
        }
      }

      // 长按处理
      if (key1_pressed)
      {
        key1_long_press_cnt++;

        // 达到长按阈值且重复计数器达到触发条件
        if (key1_long_press_cnt >= KEY_LONG_PRESS_TIME)
        {
          key1_repeat_cnt++;
          if (key1_repeat_cnt >= KEY_LONG_PRESS_REPEAT)
          {
            key1_repeat_cnt = 0;

            // 长按时的操作（与短按相同，但可以反复执行）
            // 在主菜单中
            if (menu_level == 0)
            {
              // 限制在Target和Reality之间切换
              if (current_cursor <= 1)
              {
                current_cursor = (current_cursor == 0) ? 1 : 0;
                ShowCursor(current_cursor);
              }
            }
            // 在Target页面中
            else if (menu_level == 1 && current_cursor == 0)
            {
              if (hot1_target < 80)
              {
                hot1_target++;                        // 限制最大温度
                OLED_ShowNumberSelector(hot1_target); // 更新数字选择器
              }
            }
          }
        }
      }
    }
    else
    {
      if (key1_pressed)
      {
        key1_pressed = 0;
      }
      key1_filter = 0;
      key1_long_press_cnt = 0;
      key1_repeat_cnt = 0;
    }

    // 修改KEY2按键处理，添加长按功能
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
      key2_filter++;
      if (key2_filter >= KEY_FILTER_TIME && !key2_pressed)
      {
        key2_pressed = 1;
        key2_long_press_cnt = 0;
        key2_repeat_cnt = 0;

        // 短按操作处理（保持原有逻辑）
        if (menu_level == 0)
        {
          if (current_cursor <= 1)
          {
            current_cursor = (current_cursor == 0) ? 1 : 0;
            ShowCursor(current_cursor);
          }
        }
        else if (menu_level == 1 && current_cursor == 0)
        {
          if (hot1_target > 20)
            hot1_target--;                      // 限制最小温度
          OLED_ShowNumberSelector(hot1_target); // 更新数字选择器
        }
      }

      // 长按处理
      if (key2_pressed)
      {
        key2_long_press_cnt++;

        if (key2_long_press_cnt >= KEY_LONG_PRESS_TIME)
        {
          key2_repeat_cnt++;
          if (key2_repeat_cnt >= KEY_LONG_PRESS_REPEAT)
          {
            key2_repeat_cnt = 0;

            if (menu_level == 0)
            {
              if (current_cursor <= 1)
              {
                current_cursor = (current_cursor == 0) ? 1 : 0;
                ShowCursor(current_cursor);
              }
            }
            else if (menu_level == 1 && current_cursor == 0)
            {
              if (hot1_target > 20)
              {
                hot1_target--;                        // 限制最小温度
                OLED_ShowNumberSelector(hot1_target); // 更新数字选择器
              }
            }
          }
        }
      }
    }
    else
    {
      if (key2_pressed)
      {
        key2_pressed = 0;
      }
      key2_filter = 0;
      key2_long_press_cnt = 0;
      key2_repeat_cnt = 0;
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

      // 打印计算的电阻值
      NTC1_R = ADC_GetResistance(filtered_adc4);

      // 计算温度值
      hot1_reality = Resistance2Temerature(NTC1_R);

      // // 更新PID控制
      PWM1 = (uint16_t)PID_calculate(&PID_Ctrl1, hot1_target, hot1_reality);

      // // 更新PWM输出
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM1);

      // 更新曲线数据和显示
      if (is_reality_page)
      {
        curve_update_cnt++;
        if (curve_update_cnt >= CURVE_UPDATE_PERIOD)
        {
          curve_update_cnt = 0;

          // 更新历史数据
          temp_history[curve_index] = hot1_reality;
          target_history[curve_index] = hot1_target;
          curve_index = (curve_index + 1) % CURVE_POINTS;

          // 重新绘制温度曲线
          OLED_DrawTempCurve();
        }
      }
    }
    // 串口打印控制
    print_tick++;
    if (print_tick >= 33) // 约1秒打印一次
    {
      print_tick = 0; // 重置打印计数器
      sprintf(message, "Temperature1: %.2f,%d\r\n", hot1_reality, hot1_target);
      HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); // 发送数据到串口
      sprintf(message, "current: %d,%d\r\n", adc_buf[0], adc_buf[1]);
      HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); // 发送数据到串口
    }
  }
}

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
