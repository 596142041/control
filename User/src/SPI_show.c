#include "SPI_show.h"
#include "Sys_delay.h"

u8 seg_table[] = {
    0x3f, 0x06, 0x5b, 0x4f,
    0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c,
    0x39, 0x5e, 0x79, 0x71
};

u8 fseg[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00, 0xFF}; //// 0-9 +空白
u8 fseg1[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, //(0~9)
              0xbf, 0x86, 0xdb, 0xcf, 0xe6, 0xed, 0xfd, 0x87, 0xff, 0xef,   //(0.~9.)
              0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x40, 0x80, 0x3E
             };                //(a~f - . U)
u8 fsega[] = {0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x40, 0x80, 0x3E};   //a~f
u8 dig[8] = {0xef, 0xdf, 0xbf, 0x7f, 0xfe, 0xfd, 0xfb, 0xf7};
GPIO_TypeDef *SpiSendPort[3] = {GPIOC, GPIOC, GPIOC};
GPIO_TypeDef *SegPort[3] = {GPIOE, GPIOF, GPIOF};
u16 flash_time = 0;
u16 flash_value = 0;
u16 SpiSendPin[3] = {RCLK, SRCLK, SDI};
u16 SegPin[3] = {Seg1, Seg2, Seg3};
u8 LEDtemp[3] = {0};
void GPIO_Display_Init(void)
{
    u8 axis = 0;
    GPIO_InitTypeDef GPIO_InitStructure;

    //RCC_AHBPeriphClockCmd(RCC_GPIO_SEND|RCC_GPIO_LABEL,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF, ENABLE);

    for(axis = 0; axis < 3; axis++)
    {
        //脉冲/方向/使能/刹车  电机控制端口设置

        GPIO_InitStructure.GPIO_Pin = SpiSendPin[axis] ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
        GPIO_Init(SpiSendPort[axis], &GPIO_InitStructure);
        //En
        GPIO_InitStructure.GPIO_Pin = SegPin[axis] ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
        GPIO_Init(SegPort[axis], &GPIO_InitStructure);

    }
    Seg1_Close;
    Seg2_Close;
    Seg3_Close;
    S_CLK_0;
    R_CLK_0;
    MOSIO_0;

}


void HC595SendData(unsigned char SendVal)  //
{
    unsigned char i;

    for(i = 0; i < 8; i++)
    {
        if((SendVal << i) & 0x80)
            MOSIO_1;
        else MOSIO_0;
        S_CLK_0;
        delay_us(2);
        S_CLK_1;
    }
    R_CLK_0;
    delay_us(5);
    R_CLK_1;
    //    delay_us(10);
    //   R_CLK_0;
}
void Mul_HC595SendData(u8 Chipnum, u8 *Data_buf)
{
    u8 i = 0;
    u8 Data_buf_temp;
    R_CLK_0;
    for(Chipnum = 0; Chipnum < 3; Chipnum++)
    {
        HC595Seg(Chipnum, 1);
        Data_buf_temp = *Data_buf;
        for(i = 0; i < 8; i++)
        {
            S_CLK_0;
            if((Data_buf_temp << i) & 0x80)
                MOSIO_1;
            else MOSIO_0;
            delay_us(1);
            S_CLK_1;
            delay_us(1);
        }
        Data_buf++;

    }
    R_CLK_1;
    delay_us(1);
    R_CLK_0;
}

void HC595Seg(u8 num, u8 status)
{
    if(status)
    {
        switch(num)
        {
        case 0x01: {
            Seg1_Open;
        }
        break;
        case 0x02: {
            Seg2_Open;
        }
        break;
        case 0x03: {
            Seg3_Open;
        }
        break;
        default:
            break;

        }
    }
    else if(status == 0)
    {

        switch(num)
        {
        case 0x01: {
            Seg1_Close;
        }
        break;
        case 0x02: {
            Seg2_Close;
        }
        break;
        case 0x03: {
            Seg3_Close;
        }
        break;
        default:
            break;
        }
    }
}
void LED_SPI_Send(u16 SendData)   //显示错误代码
{
    flash_time = 0;
    LEDtemp[0] = fsega[4];//字符E
    LEDtemp[1] = fseg[SendData / 10 % 10];
    LEDtemp[2] = fseg[SendData % 10];
}

void LED_SPI_Send1(u16 SendData,u16 SendData1)   //显示错误代码
{
    flash_time = 0;
    LEDtemp[0] = fsega[4];//字符E
	  
    LEDtemp[1] = fseg1[SendData ];
    LEDtemp[2] = fseg1[SendData1 ];
}
