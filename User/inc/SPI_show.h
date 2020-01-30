
#ifndef __SPI_SHOW_H
#define __SPI_SHOW_H

#include "stm32f10x.h"




#define Port_SEND        GPIOC              
#define RCLK   	GPIO_Pin_1
#define SRCLK   GPIO_Pin_0
#define SDI     GPIO_Pin_2

               
#define Port_Seg         GPIOE 
#define Port_Seg1        GPIOF

#define Seg1   GPIO_Pin_6
#define Seg2   GPIO_Pin_0
#define Seg3   GPIO_Pin_1



#define MOSIO_1     GPIO_SetBits(Port_SEND,SDI)  //
#define MOSIO_0     GPIO_ResetBits(Port_SEND,SDI)

#define R_CLK_1     GPIO_SetBits(Port_SEND,RCLK)  //
#define R_CLK_0     GPIO_ResetBits(Port_SEND,RCLK)

#define S_CLK_1     GPIO_SetBits(Port_SEND,SRCLK) //
#define S_CLK_0     GPIO_ResetBits(Port_SEND,SRCLK)

#define Seg1_Open   GPIO_SetBits(Port_Seg,Seg1) //
#define Seg1_Close  GPIO_ResetBits(Port_Seg,Seg1)

#define Seg2_Open   GPIO_SetBits(Port_Seg1,Seg2) //
#define Seg2_Close  GPIO_ResetBits(Port_Seg1,Seg2)

#define Seg3_Open   GPIO_SetBits(Port_Seg1,Seg3) //
#define Seg3_Close  GPIO_ResetBits(Port_Seg1,Seg3)
extern u8 LEDtemp[3];
extern u16 flash_time;
extern u16 flash_value;
void GPIO_Display_Init(void);
void HC595SendData(unsigned char SendVal);  //
void Mul_HC595SendData(u8 Chipnum,u8 *Data_buf);
void HC595Seg(u8 num,u8 status);
extern void LED_SPI_Send(u16 SendData);
#endif
