#ifndef __LCD_H
#define __LCD_H
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "stdlib.h"
#include "ugui.h"
#define USE_HORIZONTAL 2 // 设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif

//-----------------OLED端口定义----------------
#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOB, RES_Pin, GPIO_PIN_RESET) // RES
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOB, RES_Pin, GPIO_PIN_SET)

#define OLED_DC_Clr() HAL_GPIO_WritePin(GPIOB, DC_Pin, GPIO_PIN_RESET) // DC
#define OLED_DC_Set() HAL_GPIO_WritePin(GPIOB, DC_Pin, GPIO_PIN_SET)

#define OLED_CS_Clr() HAL_GPIO_WritePin(GPIOB, CS_Pin, GPIO_PIN_RESET) // CS
#define OLED_CS_Set() HAL_GPIO_WritePin(GPIOB, CS_Pin, GPIO_PIN_SET)

#define OLED_BLK_Clr() HAL_GPIO_WritePin(GPIOB, BLK_Pin, GPIO_PIN_RESET) // BLK
#define OLED_BLK_Set() HAL_GPIO_WritePin(GPIOB, BLK_Pin, GPIO_PIN_SET)

#define OLED_CMD 0  // 写命令
#define OLED_DATA 1 // 写数据

extern uint16_t BACK_COLOR; // 背景色

void LCD_Writ_Bus(uint8_t dat);
void LCD_WR_DATA8(uint8_t dat);
void LCD_WR_DATA(uint16_t dat);
void LCD_WR_REG(uint8_t dat);
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void Lcd_Init(void);
void LCD_Clear(uint16_t Color);
void LCD_ShowChinese(uint16_t x, uint16_t y, uint8_t index, uint8_t size, uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawPoint_big(uint16_t x, uint16_t y, uint16_t color);
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t mode, uint16_t color);
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t color);
uint32_t mypow(uint8_t m, uint8_t n);
void LCD_ShowNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t color);
void LCD_ShowNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t color);
void LCD_ShowPicture(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_ShowBattey(uint16_t x, uint16_t y, uint8_t mode);
void LCD_Show_Power(uint16_t x, uint16_t y, uint8_t mode, uint16_t power);

// 画笔颜色
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40 // 棕色
#define BRRED 0XFC07 // 棕红色
#define GRAY 0X8430  // 灰色
// GUI颜色

#define DARKBLUE 0X01CF  // 深蓝色
#define LIGHTBLUE 0X7D7C // 浅蓝色
#define GRAYBLUE 0X5458  // 灰蓝色
// 以上三色为PANEL的颜色

#define LIGHTGREEN 0X841F // 浅绿色
#define LGRAY 0XC618      // 浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE 0XA651 // 浅灰蓝色(中间层颜色)
#define LBBLUE 0X2B12    // 浅棕蓝色(选择条目的反色)



#endif
