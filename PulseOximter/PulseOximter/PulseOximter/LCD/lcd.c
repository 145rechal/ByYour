#include "lcd.h"
#include "oledfont.h"
#include "bmp.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"
uint16_t BACK_COLOR; // 背景色

/******************************************************************************
	  函数说明：LCD串行数据写入函数
	  入口数据：dat  要写入的串行数据
	  返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat)
{
	// uint8_t i;
	// OLED_CS_Clr();
	// for (i = 0; i < 8; i++)
	// {
	// 	OLED_SCLK_Clr();
	// 	if (dat & 0x80)
	// 		OLED_SDIN_Set();
	// 	else
	// 		OLED_SDIN_Clr();
	// 	OLED_SCLK_Set();
	// 	dat <<= 1;
	// }
	// OLED_CS_Set();
	OLED_CS_Clr();
	HAL_SPI_Transmit(&hspi1, &dat, 1, 1000);
	OLED_CS_Set();
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	OLED_DC_Set(); // 写数据
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
	OLED_DC_Set(); // 写数据
	LCD_Writ_Bus(dat >> 8);
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入命令
	  入口数据：dat 写入的命令
	  返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	OLED_DC_Clr(); // 写命令
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：设置起始和结束地址
	  入口数据：x1,x2 设置列的起始和结束地址
				y1,y2 设置行的起始和结束地址
	  返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if (USE_HORIZONTAL == 0)
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1 + 26);
		LCD_WR_DATA(x2 + 26);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1 + 1);
		LCD_WR_DATA(y2 + 1);
		LCD_WR_REG(0x2c); // 储存器写
	}
	else if (USE_HORIZONTAL == 1)
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1 + 26);
		LCD_WR_DATA(x2 + 26);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1 + 1);
		LCD_WR_DATA(y2 + 1);
		LCD_WR_REG(0x2c); // 储存器写
	}
	else if (USE_HORIZONTAL == 2)
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1 + 1);
		LCD_WR_DATA(x2 + 1);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1 + 26);
		LCD_WR_DATA(y2 + 26);
		LCD_WR_REG(0x2c); // 储存器写
	}
	else
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1 + 1);
		LCD_WR_DATA(x2 + 1);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1 + 26);
		LCD_WR_DATA(y2 + 26);
		LCD_WR_REG(0x2c); // 储存器写
	}
}

/******************************************************************************
	  函数说明：LCD初始化函数
	  入口数据：无
	  返回值：  无
******************************************************************************/
void Lcd_Init(void)
{
	// GPIO_InitTypeDef GPIO_InitStructure;
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); // 使能A端口时钟
	// GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_15;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度50MHz
	// GPIO_Init(GPIOA, &GPIO_InitStructure);			  // 初始化GPIOA
	// GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_15);
	OLED_RST_Set();
	OLED_DC_Set();
	OLED_CS_Set();
	OLED_BLK_Set();

	OLED_RST_Clr();
	HAL_Delay(200);
	OLED_RST_Set();
	HAL_Delay(20);
	OLED_BLK_Set();

	LCD_WR_REG(0x11);
	HAL_Delay(100);

	LCD_WR_REG(0x21);

	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB4);
	LCD_WR_DATA8(0x03);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x62);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x04);

	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0xC0);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x6A);

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);

	LCD_WR_REG(0xC5); /*VCOM*/
	LCD_WR_DATA8(0x0E);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x37);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0F);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0x36);
	if (USE_HORIZONTAL == 0)
		LCD_WR_DATA8(0x08);
	else if (USE_HORIZONTAL == 1)
		LCD_WR_DATA8(0xC8);
	else if (USE_HORIZONTAL == 2)
		LCD_WR_DATA8(0x78);
	else
		LCD_WR_DATA8(0xA8);

	LCD_WR_REG(0x29);
}

/******************************************************************************
	  函数说明：LCD清屏函数
	  入口数据：无
	  返回值：  无
******************************************************************************/
void LCD_Clear(uint16_t Color)
{
	uint16_t i, j;
	LCD_Address_Set(0, 0, LCD_W - 1, LCD_H - 1);
	for (i = 0; i < LCD_W; i++)
	{
		for (j = 0; j < LCD_H; j++)
		{
			LCD_WR_DATA(Color);
		}
	}
}

/******************************************************************************
	  函数说明：LCD显示汉字
	  入口数据：x,y   起始坐标
				index 汉字的序号
				size  字号
	  返回值：  无
******************************************************************************/
void LCD_ShowChinese(uint16_t x, uint16_t y, uint8_t index, uint8_t size, uint16_t color)
{
	uint8_t i, j;
	uint8_t *temp, size1;
	if (size == 16)
	{
		temp = Hzk16;
	} // 选择字号
	if (size == 32)
	{
		temp = Hzk32;
	}
	LCD_Address_Set(x, y, x + size - 1, y + size - 1); // 设置一个汉字的区域
	size1 = size * size / 8;						   // 一个汉字所占的字节
	temp += index * size1;							   // 写入的起始位置
	for (j = 0; j < size1; j++)
	{
		for (i = 0; i < 8; i++)
		{
			if ((*temp & (1 << i)) != 0) // 从数据的低位开始读
			{
				LCD_WR_DATA(color); // 点亮
			}
			else
			{
				LCD_WR_DATA(BACK_COLOR); // 不点亮
			}
		}
		temp++;
	}
}

/******************************************************************************
	  函数说明：LCD显示汉字
	  入口数据：x,y   起始坐标
	  返回值：  无
******************************************************************************/
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_Address_Set(x, y, x, y); // 设置光标位置
	LCD_WR_DATA(color);
}

/******************************************************************************
	  函数说明：LCD画一个大的点
	  入口数据：x,y   起始坐标
	  返回值：  无
******************************************************************************/
void LCD_DrawPoint_big(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_Fill(x - 1, y - 1, x + 1, y + 1, color);
}

/******************************************************************************
	  函数说明：在指定区域填充颜色
	  入口数据：xsta,ysta   起始坐标
				xend,yend   终止坐标
	  返回值：  无
******************************************************************************/
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
	uint16_t i, j;
	LCD_Address_Set(xsta, ysta, xend, yend); // 设置光标位置
	for (i = ysta; i <= yend; i++)
	{
		for (j = xsta; j <= xend; j++)
			LCD_WR_DATA(color); // 设置光标位置
	}
}

/******************************************************************************
	  函数说明：画线
	  入口数据：x1,y1   起始坐标
				x2,y2   终止坐标
	  返回值：  无
******************************************************************************/
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	delta_x = x2 - x1; // 计算坐标增量
	delta_y = y2 - y1;
	uRow = x1; // 画线起点坐标
	uCol = y1;
	if (delta_x > 0)
		incx = 1; // 设置单步方向
	else if (delta_x == 0)
		incx = 0; // 垂直线
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0)
		incy = 1;
	else if (delta_y == 0)
		incy = 0; // 水平线
	else
	{
		incy = -1;
		delta_y = -delta_x;
	}
	if (delta_x > delta_y)
		distance = delta_x; // 选取基本增量坐标轴
	else
		distance = delta_y;
	for (t = 0; t < distance + 1; t++)
	{
		LCD_DrawPoint(uRow, uCol, color); // 画点
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

/******************************************************************************
	  函数说明：画矩形
	  入口数据：x1,y1   起始坐标
				x2,y2   终止坐标
	  返回值：  无
******************************************************************************/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	LCD_DrawLine(x1, y1, x2, y1, color);
	LCD_DrawLine(x1, y1, x1, y2, color);
	LCD_DrawLine(x1, y2, x2, y2, color);
	LCD_DrawLine(x2, y1, x2, y2, color);
}

/******************************************************************************
	  函数说明：画圆
	  入口数据：x0,y0   圆心坐标
				r       半径
	  返回值：  无
******************************************************************************/
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int a, b;
	int di;
	a = 0;
	b = r;
	while (a <= b)
	{
		LCD_DrawPoint(x0 - b, y0 - a, color); // 3
		LCD_DrawPoint(x0 + b, y0 - a, color); // 0
		LCD_DrawPoint(x0 - a, y0 + b, color); // 1
		LCD_DrawPoint(x0 - a, y0 - b, color); // 2
		LCD_DrawPoint(x0 + b, y0 + a, color); // 4
		LCD_DrawPoint(x0 + a, y0 - b, color); // 5
		LCD_DrawPoint(x0 + a, y0 + b, color); // 6
		LCD_DrawPoint(x0 - b, y0 + a, color); // 7
		a++;
		if ((a * a + b * b) > (r * r)) // 判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
	  函数说明：显示字符
	  入口数据：x,y    起点坐标
				num    要显示的字符
				mode   1叠加方式  0非叠加方式
	  返回值：  无
******************************************************************************/
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t mode, uint16_t color)
{
	uint8_t temp;
	uint8_t pos, t;
	uint16_t x0 = x;
	if (x > LCD_W - 16 || y > LCD_H - 16)
		return;									  // 设置窗口
	num = num - ' ';							  // 得到偏移后的值
	LCD_Address_Set(x, y, x + 8 - 1, y + 16 - 1); // 设置光标位置
	if (!mode)									  // 非叠加方式
	{
		for (pos = 0; pos < 16; pos++)
		{
			temp = asc2_1608[(uint16_t)num * 16 + pos]; // 调用1608字体
			for (t = 0; t < 8; t++)
			{
				if (temp & 0x01)
					LCD_WR_DATA(color);
				else
					LCD_WR_DATA(BACK_COLOR);
				temp >>= 1;
				x++;
			}
			x = x0;
			y++;
		}
	}
	else // 叠加方式
	{
		for (pos = 0; pos < 16; pos++)
		{
			temp = asc2_1608[(uint16_t)num * 16 + pos]; // 调用1608字体
			for (t = 0; t < 8; t++)
			{
				if (temp & 0x01)
					LCD_DrawPoint(x + t, y + pos, color); // 画一个点
				temp >>= 1;
			}
		}
	}
}

/******************************************************************************
	  函数说明：显示字符串
	  入口数据：x,y    起点坐标
				*p     字符串起始地址
	  返回值：  无
******************************************************************************/
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t color)
{
	while (*p != '\0')
	{
		if (x > LCD_W - 16)
		{
			x = 0;
			y += 16;
		}
		if (y > LCD_H - 16)
		{
			y = x = 0;
			LCD_Clear(RED);
		}
		LCD_ShowChar(x, y, *p, 0, color);
		x += 8;
		p++;
	}
}

/******************************************************************************
	  函数说明：显示数字
	  入口数据：m底数，n指数
	  返回值：  无
******************************************************************************/
uint32_t mypow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}

/******************************************************************************
	  函数说明：显示数字
	  入口数据：x,y    起点坐标
				num    要显示的数字
				len    要显示的数字个数
	  返回值：  无
******************************************************************************/
void LCD_ShowNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t color)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++)
	{
		temp = (num / mypow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				LCD_ShowChar(x + 8 * t, y, ' ', 0, color);
				continue;
			}
			else
				enshow = 1;
		}
		LCD_ShowChar(x + 8 * t, y, temp + 48, 0, color);
	}
}

/******************************************************************************
	  函数说明：显示小数
	  入口数据：x,y    起点坐标
				num    要显示的小数
				len    要显示的数字个数
	  返回值：  无
******************************************************************************/
void LCD_ShowNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t color)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	uint16_t num1;
	num1 = num * 100;
	for (t = 0; t < len; t++)
	{
		temp = (num1 / mypow(10, len - t - 1)) % 10;
		if (t == (len - 2))
		{
			LCD_ShowChar(x + 8 * (len - 2), y, '.', 0, color);
			t++;
			len += 1;
		}
		LCD_ShowChar(x + 8 * t, y, temp + 48, 0, color);
	}
}

/******************************************************************************
	  函数说明：显示40x40图片
	  入口数据：x,y    起点坐标
	  返回值：  无
******************************************************************************/
void LCD_ShowPicture(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int i;
	LCD_Address_Set(x1, y1, x2, y2);
	for (i = 0; i < 1600; i++)
	{
		LCD_WR_DATA8(image[i * 2 + 1]);
		LCD_WR_DATA8(image[i * 2]);
	}
}

/******************************************************************************
	  函数说明：显示电池图标 和剩余电量
	  入口数据：x,y起点坐标
								fc 图标颜色
								fb 图标背景色
								mode 1：横屏 2：竖屏
								power 电池剩余电量，如67 55
	  返回值：  无
******************************************************************************/
void LCD_ShowBattey(uint16_t x, uint16_t y, uint8_t mode)
{
	uint8_t temp, width, length, t;
	uint16_t const_x = x;
	width = 56;
	length = 24;

	uint16_t i, TypefaceNum; // 一个字符所占字节大小
	uint16_t x0 = x;
	TypefaceNum = (width / 8 + ((width % 8) ? 1 : 0)) * length;
	// num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x, y, x + width - 1, y + length - 1); // 设置光标位置
	for (i = 0; i < TypefaceNum; i++)
	{
		temp = battery[i];
		for (t = 0; t < 8; t++)
		{
			if (temp & (0x01 << t))
			{
				if (mode == 2)
					LCD_DrawPoint(y, ((2 * const_x) - x), BLUE); // 画一个点
				else
					LCD_DrawPoint(x, y, BLUE);
			}
			x++;
			if ((x - x0) == width)
			{
				x = x0;
				y++;
				break;
			}
		}
	}
}

/******************************************************************************
	  函数说明：显示电池剩余电量
	  入口数据：x,y起点坐标
								mode 1：横屏 2：竖屏
								power 电池剩余电量，如67 55
	  返回值：  无
******************************************************************************/
void LCD_Show_Power(uint16_t x, uint16_t y, uint8_t mode, uint16_t power)
{
	uint16_t power_x_start, power_y_start, power_x_end, power_y_end;

	uint16_t fc = GREEN;
	if (power <= 20)
		fc = RED; // 低电量自动变色

	if (mode == 2)
	{
		power_x_start = y + 6;
		power_x_end = y + 19;
		power_y_start = x - 10 - ((power * 40) / 100);
		power_y_end = x - 10;
		if (power <= 100 && power > 0)
		{
			LCD_Fill(power_x_start - 1, x - 46, power_x_end, power_y_start, BLACK);
			LCD_Fill(power_x_start - 1, power_y_start, power_x_end, power_y_end + 1, fc);
		}
	}
	else
	{
		power_x_start = x + 11;
		power_x_end = x + 11 + (power * 40) / 100;
		power_y_start = y + 5;
		power_y_end = y + 19;
		if (power <= 100 && power > 0)
		{
			LCD_Fill(power_x_end, power_y_start, x + 46, power_y_end + 1, BLACK);
			LCD_Fill(power_x_start - 1, power_y_start, power_x_end, power_y_end + 1, fc);
		}
	}
}
