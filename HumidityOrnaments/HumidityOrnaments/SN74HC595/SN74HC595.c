/**
 * @file SN74HC595.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-11-14
 * @attention
 * PB2 SIG_RCK 储存寄存器时钟输入，上升沿时移位寄存器的数据进入存储寄存器，下降沿时存储寄存器数据不变
 * PB0 SIG_SCK 移位寄存器时钟输入，该引脚上升沿时数据寄存器的输出进行移位
 * PA5 SIG_SER 串行数据输入
 *     SIG_CLK 硬件10k上拉
 *     SIG_OE  硬件10k下拉，默认使能
 *
 * 平常把这SCK/RCK两个引脚的电平置低，需要移位的时候就把SCK拉高一下，需要锁存的时候将RCK拉高一下。
 *
 */
#include "SN74HC595.h"

extern uint8_t LED6[6];       // 显示缓存
extern uint8_t display_index; // 显示位索引

/*取非关系
共阴二极管---dp g f e d c b a
0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x7b,0x71
共阳二极管---dp g f e d c b a
0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x84,0x8e
*/

/*标准字库*/
uint8_t t_display[] = {
    // 0    1    2      3     4    5     6     7     8     9      A     B     C     D    E     F
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x7b, 0x71,
    // black -    H    J     K     L      N     O     P     U    T     G     Q      R    M     Y
    0x00, 0x40, 0x76, 0x1e, 0x70, 0x38, 0x37, 0x5c, 0x73, 0x3e, 0x78, 0x3d, 0x67, 0x50, 0x37, 0x6e,
    // 0.   1.   2.   3.     4.    5.    6.    7.    8.    9.   -1
    0xbf, 0x86, 0xdb, 0xcf, 0xe6, 0xed, 0xfd, 0x87, 0xff, 0xef, 0x46};

/*位码*/
uint8_t bit_code[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

extern TIM_HandleTypeDef htim3;

void delay_us(uint32_t us)
{
    uint16_t differ = 0xffff - us - 5;
    /*为了防止因中断打断延时，造成计数错误，如从0xfffe开始延时1us,但由于中断打断(此时计数器仍在计数),本因计数到0xffff便停止计数，但是由于错过计数值，并重载ARR值，导致实际延时(0xffff+1)us*/
    HAL_TIM_Base_Start(&htim3);
    __HAL_TIM_SetCounter(&htim3, differ);
    while (differ < 0xffff - 5)
    {
        differ = __HAL_TIM_GetCounter(&htim3);
    }
    HAL_TIM_Base_Stop(&htim3);
}

/*将数据写入74HC595中*/
void SN74HC595_Write(uint8_t data)
{
    uint8_t i, temp;
    temp = data;
    for (i = 0; i < 8; i++)
    {
        SCK_L();         // 拉低SCK时钟
        if (temp & 0x80) // SER发送数据
        {
            SER_H();
        }
        else
        {
            SER_L();
        }
        temp = temp << 1;
        SCK_H(); // SCK拉高产生时钟信号
        __nop();
    }
    __nop();
    __nop();
    SCK_L();
}

/*将数据写入到寄存器的数据输出到端口显示*/
void SN74HC595_Out(void)
{
    RCK_L();
    __nop();
    RCK_H();
    __nop();
    __nop();
    RCK_L();
}

/*初始化595的输出为0，消影用*/
void SN74HC595_RST(void)
{
    uint8_t i;
    for (i = 0; i > 6; i++)
    {
        SN74HC595_Write(0);
        SN74HC595_Write(0);
    }
    SN74HC595_Out();
}

/*恢复暂态值为消影值*/
void LED6_RST(void)
{
    LED6[0] = 0x10; // 该位SIG消影
    LED6[1] = 0x10; // 该位SIG消影
    LED6[2] = 0x10; // 该位SIG消影
    LED6[3] = 0x10; // 该位SIG消影
    LED6[4] = 0x10; // 该位SIG消影
    LED6[5] = 0x10; // 该位SIG消影
}

/************显示扫描函数**********/
void Display_Scan(void)
{
    /*注意按照原理图的设计，先写段再写位*/
    /*共阴极*/
    SN74HC595_Write(t_display[LED6[display_index]]); /*写段*/
    SN74HC595_Write(~bit_code[display_index]);       /*写位*/
    /*共阳极*/
    // SN74HC595_Write(~t_display[LED6[display_index]]); /*写段*/
    // SN74HC595_Write(bit_code[display_index]);       /*写位*/
    SN74HC595_Out();
    delay_us(50); // 延时50us
    if (++display_index >= 6)
    {
        display_index = 0;
    }
}
