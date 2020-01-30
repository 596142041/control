#include "Comm_IO.h"
#include "stm32f10x.h"
#include "main.h"
#include "sys_delay.h"

u16 KG_Time = 200;
u8 BG_ZS_Flag = 0;
u16 KeyScan_Times = 0;
u16 KeyScan_Delay = 0;
u8 KeyValue[14] = {0};
u8 KeyValue_old[14] = {0};
u16 status_all_old = 0;
u16 status_all_next = 0;
u8 ys=0;
u8 dlg=0;
u8 SZ1_flag = 0;
u8 SZ2_flag = 0;
u8 YK1_flag = 0;
u32 YK1_time_flag = 0; //计时
u8 YK1_count_flag = 0; //计数
u8 YK2_flag = 0;
u8 BG_finish_flag = 0;
u8 PC_Switch_flag = 0;
u16 Expose_time_flag = 0;
u16 time_flag1 = 0;
u16 SZ1_time_flag=0;
u16 SZ1_time_flag0=0;
u32 a1=0;
u8 open_time = 0;
//////////////////////////////////////说明：电源控制端口
GPIO_TypeDef *PowerinPort[4] = {GPIOA, GPIOC, GPIOE, GPIOE,};
u16 PowerinPin[4] = {CE_FB, KE_FB, ST_FB, HV_FB};
GPIO_TypeDef *PoweroutPort[4] = {GPIOA, GPIOE, GPIOF, GPIOF};
u16 PoweroutPin[4] = {HV_EN, WHD_EN, IPC_EN, Motor_EN};
///////////////////////////////////////////////////////////
//////////////////////////////////////说明：曝光控制端口
GPIO_TypeDef *exposeinPort[4] = {GPIOE, GPIOE, GPIOD, GPIOD};
u16 exposeinPin[4] = {SZ1, SZ2, YK1, YK2};

GPIO_TypeDef *exposeoutPort[5] = {GPIOD, GPIOD, GPIOD, GPIOD, GPIOE};
u16 exposeoutPin[5] = {RGO1, RGO2, BG1, BG2, Charge_sw};
///////////////////////////////////////////////////////////
//////////////////////////////////////说明：开机控制端口
GPIO_TypeDef *OpeninPort[8] = {GPIOB, GPIOA, GPIOG, GPIOG, GPIOG, GPIOG, GPIOA, GPIOC};
u16 OpeninPin[8] = {PC_Button, PC_FB, JN_FB, YL_FB1, YL_FB2, PB_FB1, PB_FB2, PB_FB3};

GPIO_TypeDef *OpenoutPort[9] = {GPIOB, GPIOB, GPIOB, GPIOG, GPIOG, GPIOE, GPIOC, GPIOC, GPIOC};
u16 OpenoutPin[9] = {PC_EN, LCD_EN, Light_EN, TB_JN, CC_EN, Charge_inter, PB_EN1, PB_EN2, PB_EN3};
///////////////////////////////////////////////////////////
//////////////////////////////////////说明：显示控制端口
GPIO_TypeDef *showinPort[2] = {GPIOG, GPIOD};
u16 showinPin[2] = {BG_FB, HVG_open};
GPIO_TypeDef *showoutPort[5] = {GPIOE, GPIOE, GPIOE, GPIOE, GPIOD};
u16 showoutPin[5] = {Beep, Red_EN, Green_EN, Blue_EN, HVG_EN};
///////////////////////////////////////////////////////////
//#define RS_status_BG_FB 	(u16)GPIO_ReadInputDataBit(showinPort[0],showinPin[0])


/*************************************************************
函数名称:Turn_LED_Init
功    能:LED初始化
参    数:无
返 回 值:无
说    明:无
*************************************************************/

void Turn_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void MCU_BEEP_Config(void )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void MCU_BEEP_SET(u8 status)
{

    if(0 == status)
    {
        GPIO_ResetBits(GPIOE, GPIO_Pin_5);
    }
    else if (1 == status)
    {
        GPIO_SetBits(GPIOE, GPIO_Pin_5);
    }
}
/*************************************************************
函数名称:Turn_LED_Init
功    能:LED初始化
参    数:无
返 回 值:无
说    明:无
*************************************************************/

void Off_Power_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
void Off_Power_SET(u8 status)
{
    if(0 == status)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6);
    }
    else if (1 == status)
    {   GPIO_SetBits(GPIOA, GPIO_Pin_6);
    }
}

/*************************************************************
函数名称:LED_SET
功    能:调试LED的开关
参    数:无
返 回 值:无
说    明:无
*************************************************************/
void LED_SET(u8 led, u8 status)
{
    if(led == 1)
    {
        if(0 == status)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_4);
        }
        else if (1 == status)
        {
            GPIO_ResetBits(GPIOC, GPIO_Pin_4);
        }
    }
    if(led == 2)
    {
        if(0 == status)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_5);
        }
        else if (1 == status)
        {
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);
        }
    }
}


/*************************************************************
函数名称:COM_GPIO_Init
功    能:端口初始化
参    数:无
返 回 值:无
说    明:无
*************************************************************/



void  COM_GPIO_Init(void)
{
    u8 i = 0;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
    //power out
    for(i = 0; i < 4; i++)
    {
        GPIO_InitStructure.GPIO_Pin = PoweroutPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(PoweroutPort[i], &GPIO_InitStructure);

    }
    ////power in
    for(i = 0; i < 4; i++)
    {
        GPIO_InitStructure.GPIO_Pin = PowerinPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(PowerinPort[i], &GPIO_InitStructure);

    }
    for(i = 0; i < 5; i++) //expose out
    {
        GPIO_InitStructure.GPIO_Pin = exposeoutPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(exposeoutPort[i], &GPIO_InitStructure);

    }
    ////expose in 考虑是否用中断形式；
    /*
    for(i=0;i<4;i++)
    {
     GPIO_InitStructure.GPIO_Pin=exposeinPin[i];
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
     GPIO_Init(exposeinPort[i], &GPIO_InitStructure);

    }
    */
    for(i = 0; i < 9; i++) //open out
    {
        GPIO_InitStructure.GPIO_Pin = OpenoutPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(OpenoutPort[i], &GPIO_InitStructure);

    }
    ////open in
    for(i = 0; i < 8; i++)
    {
        GPIO_InitStructure.GPIO_Pin = OpeninPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(OpeninPort[i], &GPIO_InitStructure);

    }



    for(i = 0; i < 5; i++) //show out
    {
        GPIO_InitStructure.GPIO_Pin = showoutPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(showoutPort[i], &GPIO_InitStructure);

    }
    ////show in
    for(i = 0; i < 2; i++)
    {
        GPIO_InitStructure.GPIO_Pin = showinPin[i];
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
        GPIO_Init(showinPort[i], &GPIO_InitStructure);

    }

//
//   GPIO_InitStructure.GPIO_Pin=OpenoutPin[3];
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
//    GPIO_Init(OpenoutPort[3], &GPIO_InitStructure);
}

///////////////////////////////
/*
#define  SZ1              GPIO_Pin_15   //PE15
#define  SZ2              GPIO_Pin_14    //PE14
#define  YK1              GPIO_Pin_11  // PD11
#define  YK2              GPIO_Pin_10   //PD10
*/

void Expose_GPIO_Init(void )
{
// u8 i=0;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);


    GPIO_InitStructure.GPIO_Pin = exposeinPin[3];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(exposeinPort[3], &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = exposeinPin[2];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(exposeinPort[2], &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = exposeinPin[1];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(exposeinPort[1], &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = exposeinPin[0];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(exposeinPort[0], &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M时钟速度
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断模式，可选值为中断 EXTI_Mode_Interrupt 和事件 EXTI_Mode_Event。
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//触发方式，可以是下降沿触发 EXTI_Trigger_Falling，上升沿触发 EXTI_Trigger_Rising，或者任意电平（上升沿和下降沿）触发EXTI_Trigger_Rising_Falling
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4; //中断线的标号 取值范围为EXTI_Line0~EXTI_Line15  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断模式，可选值为中断 EXTI_Mode_Interrupt 和事件 EXTI_Mode_Event。
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//触发方式，可以是下降沿触发 EXTI_Trigger_Falling，上升沿触发 EXTI_Trigger_Rising，或者任意电平（上升沿和下降沿）触发EXTI_Trigger_Rising_Falling
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

}

void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET) //判断某个线上的中断是否发生
    {
        BG_finish_flag = 1;//曝光反馈信号上升沿触发中断BG_FB
			  MCU_BEEP_SET(0);
        EXTI_ClearITPendingBit(EXTI_Line4);   //清除 LINE 上的中断标志位
    }

}

//
//void EXTI15_10_IRQHandler(void)
//{
//  if(EXTI_GetITStatus(EXTI_Line10)!=RESET)//判断某个线上的中断是否发生
//  {
//    YK2_flag=1;
//    EXTI_ClearITPendingBit(EXTI_Line10);   //清除 LINE 上的中断标志位
//  }
//  if(EXTI_GetITStatus(EXTI_Line11)!=RESET)//判断某个线上的中断是否发生
//  {
//    YK1_flag=1;
//    EXTI_ClearITPendingBit(EXTI_Line11);   //清除 LINE 上的中断标志位
//  }
//  /*
//    if(EXTI_GetITStatus(EXTI_Line14)!=RESET)//判断某个线上的中断是否发生
//  {
//    SZ2_flag=1;
//    EXTI_ClearITPendingBit(EXTI_Line14);   //清除 LINE 上的中断标志位
//  }
//    if(EXTI_GetITStatus(EXTI_Line15)!=RESET)//判断某个线上的中断是否发生
//  {
//   //  YK2_flag=1;
//     SZ1_flag=1;
//    EXTI_ClearITPendingBit(EXTI_Line15);   //清除 LINE 上的中断标志位
//  }
//*/
//}


void Power_SET(u8 num, u8 status)
{

    if(0 == status)
    {
        GPIO_ResetBits(PoweroutPort[num], PoweroutPin[num]);
    }


    else if(1 == status)
    {
        GPIO_SetBits(PoweroutPort[num], PoweroutPin[num]);
    }

}


void Expose_SET(u8 num, u8 status)
{

    if(0 == status)
    {
        GPIO_ResetBits(exposeoutPort[num], exposeoutPin[num]);
    }
    else if(1 == status)
    {
        GPIO_SetBits(exposeoutPort[num], exposeoutPin[num]);
    }

}


void Open_SET(u8 num, u8 status)
{

    if(0 == status)
    {
        GPIO_ResetBits(OpenoutPort[num], OpenoutPin[num]);
    }
    else if(1 == status)
    {
        GPIO_SetBits(OpenoutPort[num], OpenoutPin[num]);
    }

}


void Show_SET(u8 num, u8 status)
{

    if(0 == status)
    {
        GPIO_ResetBits(showoutPort[num], showoutPin[num]);
    }
    else if(1 == status)
    {
        GPIO_SetBits(showoutPort[num], showoutPin[num]);
    }

}



///////////////////********************
//按键扫描函数
//Key_all_status[12] 存储按键值标志位；KeyScan_Times扫描次数；KeyScan_Delay消抖时间
////////////////**********************
void Key_Scan(void)
{
    if(Open_PC_Button_status == 1)
    {

        if(KeyScan_Times == 0)
        {
            KeyScan_Times = 1;
        }
        if((KeyScan_Delay > KG_Time) && (KeyScan_Times == 1)) //消抖时间10，按键扫描次数2
        {

               if(Open_PC_Button_status==1)   ////扫描按键
            {
                KeyScan_Times = 2;
                if(KeyScan_Times == 2)
                {
                    if(1 == Open_PC_FB_status)   //判断输出标示位
                    {
                           PC_Switch_flag=0;
                        open_time = 0;
                        KG_Time = 400;
                        Power_SET(2, 0); //
                    }
                    else if(0 == Open_PC_FB_status)   //判断输出标示位
                    {
                        Power_SET(2, 1);
                            PC_Switch_flag=1;
                        open_time = 1;
                        system_start_flag = 1200;
                        KG_Time = 400;
                    }             
                }
            }
        }
        if(KeyScan_Delay > KG_Time * 2 - 1)
        {
            KeyScan_Times = 3;
            if(1 == Open_PC_FB_status)
                PC_Switch_flag = 1;
            else if(0 == Open_PC_FB_status)
            {
                PC_Switch_flag = 0;
                system_start_flag = 0;
            }
        }
    }
    if(Open_PC_Button_status == 0)
    {
        KeyScan_Delay = 0;		
        KeyScan_Times = 0;
    }
    if(KeyScan_Times > 0)         ////键盘扫描
        KeyScan_Delay++;
}

void Key_Process(void)
{

}
