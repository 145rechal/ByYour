#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

#define OLED_CHN_CHAR_WIDTH			3		//UTF-8编码格式给3，GB2312编码格式给2

typedef struct 
{
	char Index[OLED_CHN_CHAR_WIDTH + 1];	//汉字索引
	uint8_t Data[28];						//字模数据
} ChineseCell_t;

extern const uint8_t OLED_F8x16[][16];
extern const uint8_t OLED_F6x8[][6];
extern const uint8_t OLED_F24x12[][36];

//extern const ChineseCell_t OLED_CF16x16[];
extern const ChineseCell_t OLED_CF14x14[];
/*图像数据声明*/
extern const uint8_t Diode[];
/*按照上面的格式，在这个位置加入新的图像数据声明*/
#endif
