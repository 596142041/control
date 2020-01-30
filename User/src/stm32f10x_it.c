/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "User_Uart.h"
#include "GYTH_CAN.h"
//#include "sys_config.h"
#include "sys_delay.h"
#include "Comm_IO.h"
#include "SPI_show.h"
#include "state.h"
extern timer sz1,beep_timer;
u8 u1_buffer[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
u8 flash_num;
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char i, ret;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)        //初始化1ms中断一次
{
    TimingDelay_Decrement();
    Counter_1mS++;

    if(YK1_time_flag > 0)
        YK1_time_flag--;

    if(Expose_time_flag != 0)
        Expose_time_flag--;
    if (time_flag1 > 0)
        time_flag1--;
		if (dlg!=0)
			 Off_Power_SET(1);
			

    flash_num++;
    if(flash_num == 1)
    {
        HC595SendData(LEDtemp[0]);
        Seg1_Open;
        Seg2_Close;
        Seg3_Close;
    }
    if(flash_num == 2)
    {
        HC595SendData(LEDtemp[1]);
        Seg2_Open;
        Seg1_Close;
        Seg3_Close;
    }
    if(flash_num == 3)
    {
        HC595SendData(LEDtemp[2]);
        Seg3_Open;
        Seg2_Close;
        Seg1_Close;
        flash_num = 0;
    }

    Key_Scan();
    if(flash_time != 0)
        flash_time--;

    if(system_start_flag > 0)
        system_start_flag--;

    if(system_start_flag != 0)
    {
        Open_SET(0, 1);   //开机信号
        Power_SET(2, 1);
    }
    if(system_start_flag == 0)
    {
        Open_SET(0, 0);   //开机信号
        open_time = 0;
    }
    if(1 == Open_PC_FB_status)//if(1 == Open_PC_FB_status)
    {
        Open_SET(2, 1);    //工控机按钮指示灯
		if(1 == Expose_SZ1_status)
		{ 
			SZ1_flag = 1;
			sz1.run = 1;
		}
        else
		{
			SZ1_flag = 0;
			sz1.run = 0;
			sz1.cnt = 0;
			sz1.update_flag = 0;
			sz1.time_out_flag = 0;
		}
		if(sz1.run == 1)
		{
			sz1.cnt++;
			if(sz1.cnt == sz1.period)
			{
				sz1.update_flag = 1;
			}
			else if(sz1.cnt == sz1.time_out)
			{
				sz1.time_out_flag = 1;
			}
		}
		if(sz1.update_flag == 1)//定时器达到
		{
			 if(1 == Expose_SZ2_status)//标示位位1
			{ 
				SZ2_flag = 1;
				Expose_SET(1,1); //开启二挡信号
				beep_timer.run = 1;
			}
			else
			{
				SZ2_flag = 0;
				Expose_SET(1,0);
				MCU_BEEP_SET(0);
				beep_timer.run = 0;
				beep_timer.update_flag = 0;
			}
		}
		else
		{
			 Expose_SET(1,0); //开启二挡信号
			beep_timer.run = 0;
			beep_timer.cnt = 0;
			beep_timer.update_flag = 0;
		}
       if(beep_timer.run == 1)
	   {
		   beep_timer.cnt++;
		   if(beep_timer.cnt == beep_timer.period)
		   {
			   beep_timer.update_flag = 1;//蜂鸣器时间到
			   // beep_timer.cnt = 0;
		   }
	   }

        //无线手闸曝光时
        if(1 == Expose_YK1_status)//遥控1档手闸信号----按下
            YK1_flag = 1;
        else//遥控1档手闸信号----弹起
				{   YK1_flag = 0;
            YK1_count_flag = 0;
        }
    }
    else if(0 == Open_PC_FB_status)
    {
        Open_SET(2, 0);    //工控机按钮指示灯

    }

 
    if(mcu_beep_time_flag > 0)
        mcu_beep_time_flag=0;
}





/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/


/**
  * @brief  This function handles CAN1 RX0 Handler.
  * @param  None
  * @retval None
  */


void USB_LP_CAN1_RX0_IRQHandler(void)///CAN通信接收中断函数
{
    Init_RxMes(&RxMessage);  //初始化
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

    if ((RxMessage.ExtId == 0x13) && (RxMessage.DLC == 8))   //接收到牛头发送的数据
    {
        ret = 1;
          //LED_Display(RxMessage.Data[0]);
           for(i=0;i<8;i++)
			{
			RXCAN_Protol2[i] =0;
            RXCAN_Protol2[i] = RxMessage.Data[i];
			}
    }else
    if ((RxMessage.ExtId == 0x12) && (RxMessage.DLC == 8))   //接受到充电数据
    {
        ret = 2;
        for(i = 0; i < 8; i++)
        {
            RXCAN_Protol[i] = 0;
            RXCAN_Protol[i] = RxMessage.Data[i];
        }
		}else
     if ((RxMessage.ExtId == 0x22) && (RxMessage.DLC == 8))   //接受到充电数据,充电板与主控板2次CAN通信
    {
        ret = 2;
        for(i = 0; i < 8; i++)
        {
            RXCAN_Protol[i] = 0;
            RXCAN_Protol[i] = RxMessage.Data[i];
        }
    }else
    if ((RxMessage.ExtId == 0x11) && (RxMessage.DLC == 8))  //接收到运动板信息
    {
        ret = 3;
        for(i = 0; i < 8; i++)
        {
            RXCAN_Protol1[i] = 0;
            RXCAN_Protol1[i] = RxMessage.Data[i];
        }
    } 
    if ((RxMessage.ExtId == 0x21) && (RxMessage.DLC == 8))  //接收到运动板应变片信息
    {
        ret = 4;
        for(i = 0; i < 8; i++)
        {
            RXCAN_Protol11[i] = 0;
            RXCAN_Protol11[i] = RxMessage.Data[i];
        }
    }
		if ((RxMessage.ExtId == 0x31) && (RxMessage.DLC == 8))  //接收到运动板卡
		{    ret=6;
			   for(i = 0; i < 8; i++)
        {
            RXCAN_Protol12[i] = 0;
            RXCAN_Protol12[i] = RxMessage.Data[i];
		     }
		}
		
}



void USART1_IRQHandler(void)
{
  u8 i = 0;
  u8 Uart_Res1; //  接收数据暂存变量
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    //   USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    /* Read one byte from the receive data register */
    Uart_Res1 = USART_ReceiveData(USART1);
    switch (Uart_Res1)
    {
    case 0x5A: //  头帧
    {
      RxCounter1 = 0;
      RxBuffer1[RxCounter1] = Uart_Res1;
      RxCounter1++;
    }
    break;
    case 0xA5: //  尾帧
    {
      if ((RxBuffer1[0] == 0x5A) && (RxCounter1 == RxBuffer1[1] - 1))
      {
        RxBuffer1[RxCounter1] = Uart_Res1;
        //  Uart_RX_Length=0;
        Uart_RX_Flag1 = 1;                              //  接收结束标示位
                                                        //  RxCounter1=0;  
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //  关闭接收使能
      }
      else
      {
        RxCounter1 = 0;
      }
    }
    break;
    default: //   数据
    {
      if ((0x5A == RxBuffer1[0]))
      {
        RxBuffer1[RxCounter1] = Uart_Res1;
        RxCounter1++;
      }
      else
      {
        //  帧出错,请求重发
        {
        }
        RxCounter1 = 0;
        for (i = 0; i < UART_RX_MAX1; i++)
          RxBuffer1[i] = 0;
      }
    }
    break;
    }
  }
  if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
  {
    USART_ClearFlag(USART1, USART_FLAG_ORE); //   读SR
    USART_ReceiveData(USART1);               //  读DR
  }
}


extern u8 computer_send_flag;    //计算机发送标示位
extern u8 Bg_flag;
void USART2_IRQHandler(void)
{
    u8 i=0;
    static  u8 len;
    u8 Uart_Res2;//数据接收暂存量
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
       // USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        /* Read one byte from the receive data register */
        Uart_Res2 = USART_ReceiveData(USART2);    //串口暂存数据
        switch (Uart_Res2)
        {
        case 0x5a: //帧头，
        {
            RxCounter2 = 0;
            RxBuffer2[RxCounter2] = Uart_Res2;//RxBuffer2[0]=0x5a;
            RxCounter2++;
        }
        break;

        default: //数据
        {
            if ((0x5a == RxBuffer2[0]))
            {
                RxBuffer2[RxCounter2] = Uart_Res2;
                RxCounter2++;
                if ((2 == RxCounter2))//RxBuffer2[1]=len;数据长度
                {
                    len = Uart_Res2;
                }
                
                if ((RxCounter2 == len + 4)&&(RxCounter2>3))//&&(RxBuffer2[2]==0x04)板号04代表发送给牛头板子
                {
                    u8 crc[2];
                    crc[0] = RxBuffer2[len + 1];
                    crc[1] = RxBuffer2[len + 2];
                    ModBusCRC16(&RxBuffer2[1], len);
                    if(crc[0] == RxBuffer2[len + 1] && crc[1] == RxBuffer2[len + 2])
                    {
                      computer_send_flag =1; 
											if(RxBuffer2[2]==0x04)
											{
						//           can_send_buff[7]=RxBuffer2[10];
												CAN_TX(&RxBuffer2[3] ,0x14);//0x14牛头地址，默认数据发送到牛头
												if(RxBuffer2[4]==0x06)
													Uart_Res2=0;
												if(RxBuffer2[4]==0x07)
													Uart_Res2=0;
												if(RxBuffer2[4]==0x08)
													Uart_Res2=0;
												if(RxBuffer2[4]==0x09)
													Uart_Res2=0;												
											}
											if(RxBuffer2[2]==0x06)//目前(RxBuffer2[2]==0x06)cmd=06强制调教应变片
											{
												CAN_TX(&RxBuffer2[3] ,0x16);//0x16牛头地址，默认数据发送到运动控制板///接收缓存,CAN地址位
											}
											if(RxBuffer2[2]==0x05)//目前(RxBuffer2[2]==0x05)cmd=05设置延时曝光参数
					//  if(Bg_flag==1)
					{ //Bg_flag=0;
						MCU_BEEP_SET(1);
						Expose_SET(0, 1);  //MCU软件一档信号发出
						//feed_dog();
						delay_ms(2000);
						//feed_dog();
						Expose_SET(1, 1);  //MCU软件二档信号发出
						delay_ms(2000);
						//feed_dog();
						MCU_BEEP_SET(0);
						Expose_SET(0, 0);  //MCU软件二档信号发出
						Expose_SET(1, 0);  //MCU软件二档信号发出
						RxBuffer2[2]=0;
					}
											//if(RxBuffer2[2]==0x05)
										//	{  Bg_flag=1;
											//}
                    }
                }


            }
        }
        break;
        }

    }

    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET)
    {
        USART_ClearFlag(USART2, USART_FLAG_ORE);	//读SR
        USART_ReceiveData(USART2);  //读DR
    }


}

void USART3_IRQHandler(void)
{
    u8 i = 0;
    u8 Uart_Res3;//数据接收暂存量
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        /* Read one byte from the receive data register */
        Uart_Res3 = USART_ReceiveData(USART3);

    }

    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) == SET)
    {
        USART_ClearFlag(USART3, USART_FLAG_ORE);	//读SR
        USART_ReceiveData(USART3);  //读DR
    }


}





void USART5_IRQHandler(void)
{
    u8 i = 0;
    u8 Uart_Res5;//数据接收暂存量
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
        /* Read one byte from the receive data register */
        Uart_Res5 = USART_ReceiveData(UART5);
    }


    if(USART_GetFlagStatus(UART5, USART_FLAG_ORE) == SET)
    {
        USART_ClearFlag(UART5, USART_FLAG_ORE);	//读SR
        USART_ReceiveData(UART5);  //读DR
    }


}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
