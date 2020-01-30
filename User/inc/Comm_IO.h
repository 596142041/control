#ifndef _COMM_IO_H_
#define _COMM_IO_H_
#include "stm32f10x.h"

extern u8 SZ1_flag;
extern u8 SZ2_flag;
extern u8 YK1_flag;
extern u32 YK1_time_flag;
extern u8 YK1_count_flag;
extern u8 YK2_flag;
extern u8 BG_finish_flag;
extern u8 ys;

extern u16 KeyScan_Times;     
extern u16 KeyScan_Delay;
extern u8 KeyValue[14];  //存储按键值，全局变量
extern u8 KeyValue_old[14];
extern  u16 status_all_old;
extern  u16 status_all_next;
extern u8 dlg;

extern u8 open_time;

//////////////
///电源控制端口
//out

#define  HV_EN          GPIO_Pin_4      //PA4  高压电源 
#define  WHD_EN         GPIO_Pin_10    //PE10  限速器电源
#define  IPC_EN         GPIO_Pin_8      //PF8  工控机电源
#define  Motor_EN        GPIO_Pin_10   //PF10   急停开关

//in
#define  CE_FB       GPIO_Pin_1   //PA1
#define  KE_FB       GPIO_Pin_3    //PC3  
#define  ST_FB       GPIO_Pin_12  // PE12
#define  HV_FB       GPIO_Pin_11   //PE11

//
/////////////////////////////////////
//曝光控制端口
//out

#define  RGO1             GPIO_Pin_8     //PD8
#define  RGO2             GPIO_Pin_9    //PD9
#define  BG1              GPIO_Pin_12      //PD12  
#define  BG2              GPIO_Pin_13   //PD13
#define  Charge_sw        GPIO_Pin_13  //PE13 

//in
#define  SZ1              GPIO_Pin_15   //PE15
#define  SZ2              GPIO_Pin_14    //PE14  
#define  YK1              GPIO_Pin_11  // PD11
#define  YK2              GPIO_Pin_10   //PD10
//
/////////////////////////////////////

/////////////////////////////////////
//开机控制端口
//out

#define  PC_EN            GPIO_Pin_8     //PB8
#define  LCD_EN           GPIO_Pin_9    //PB9
#define  Light_EN         GPIO_Pin_6      //PB
#define  TB_JN            GPIO_Pin_3    //PG3
#define  CC_EN            GPIO_Pin_7  //PG7 
#define  Charge_inter     GPIO_Pin_0     //PE0
#define  PB_EN1           GPIO_Pin_8    //PC8    
#define  PB_EN2           GPIO_Pin_9    //PC9
#define  PB_EN3           GPIO_Pin_7    //PC7 
//in
#define  PC_Button        GPIO_Pin_7   //PB7
#define  PC_FB            GPIO_Pin_5    //PA5 
#define  JN_FB            GPIO_Pin_2  // PG2
#define  YL_FB1           GPIO_Pin_6   //PG6
#define  YL_FB2           GPIO_Pin_5   //PG5
#define  PB_FB1           GPIO_Pin_8   //PG8
#define  PB_FB2           GPIO_Pin_8   //PA8
#define  PB_FB3           GPIO_Pin_6   //PC6

//
/////////////////////////////////////
/////////////////////////////////////
//显示控制端口
//out

#define  Beep             GPIO_Pin_1     //PE1
#define  Red_EN           GPIO_Pin_4    //PE4
#define  Green_EN         GPIO_Pin_3      //PE3 
#define  Blue_EN          GPIO_Pin_2   //PE2
#define  HVG_EN           GPIO_Pin_14  //PD14 

//in
#define  BG_FB       GPIO_Pin_4   //PG4
#define  HVG_open       GPIO_Pin_15    //PD15 
//
/////////////////////////////////////
//////////////////////////////////////说明：电源控制端口
extern GPIO_TypeDef *PowerinPort[4];
extern u16 PowerinPin[4];
extern GPIO_TypeDef *PoweroutPort[4];
extern u16 PoweroutPin[4];
///////////////////////////////////////////////////////////
//////////////////////////////////////说明：曝光控制端口
extern GPIO_TypeDef *exposeinPort[4];
extern u16 exposeinPin[4];
extern GPIO_TypeDef *exposeoutPort[5];
extern u16 exposeoutPin[5];
///////////////////////////////////////////////////////////
//////////////////////////////////////说明：开机控制端口
extern GPIO_TypeDef *OpeninPort[8];
extern u16 OpeninPin[8];
extern GPIO_TypeDef *OpenoutPort[9];
extern u16 OpenoutPin[9];
///////////////////////////////////////////////////////////
//////////////////////////////////////说明：显示控制端口
extern GPIO_TypeDef *showinPort[2];
extern u16 showinPin[2];
extern GPIO_TypeDef *showoutPort[5];
extern u16 showoutPin[5];
///////////////////////////////////////////////////////////
extern u8 BG_ZS_Flag;

#define Power_CE_FB_status 	(u16)GPIO_ReadInputDataBit(PowerinPort[0],PowerinPin[0])
#define Power_KE_FB_status 	(u16)GPIO_ReadInputDataBit(PowerinPort[1],PowerinPin[1])
#define Power_ST_FB_status 	(u16)GPIO_ReadInputDataBit(PowerinPort[2],PowerinPin[2])
#define Power_HV_FB_status 	(u16)GPIO_ReadInputDataBit(PowerinPort[3],PowerinPin[3])

#define Expose_SZ1_status 	(u16)GPIO_ReadInputDataBit(exposeinPort[0],exposeinPin[0])
#define Expose_SZ2_status 	(u16)GPIO_ReadInputDataBit(exposeinPort[1],exposeinPin[1])
#define Expose_YK1_status 	(u16)GPIO_ReadInputDataBit(exposeinPort[2],exposeinPin[2])
#define Expose_YK2_status 	(u16)GPIO_ReadInputDataBit(exposeinPort[3],exposeinPin[3])


#define Open_PC_Button_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[0],OpeninPin[0])
#define Open_PC_FB_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[1],OpeninPin[1])
#define Open_JN_FB_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[2],OpeninPin[2])
#define Open_YL_FB1_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[3],OpeninPin[3])
#define Open_YL_FB2_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[4],OpeninPin[4])
#define Open_PB_FB1_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[5],OpeninPin[5])
#define Open_PB_FB2_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[6],OpeninPin[6])
#define Open_PB_FB3_status 	(u16)GPIO_ReadInputDataBit(OpeninPort[7],OpeninPin[7])

#define Show_BG_FB_status 	(u16)GPIO_ReadInputDataBit(showinPort[0],showinPin[0])
#define Show_HV_en_status 	(u16)GPIO_ReadInputDataBit(showinPort[1],showinPin[1])

extern u8 PC_Switch_flag;
extern u16 Expose_time_flag;
extern u16 time_flag1;
extern u16 SZ1_time_flag;
extern u16 SZ1_time_flag0;
extern u32 a1;
void MCU_BEEP_Config(void);
void MCU_BEEP_SET(u8 status);
void Off_Power_Init(void);
void Off_Power_SET(u8 status);
void Turn_LED_Init(void);
void LED_SET(u8 led,u8 status);
void  COM_GPIO_Init(void);
void Expose_GPIO_Init(void );
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void Power_SET(u8 num,u8 status);
void Expose_SET(u8 num,u8 status);
void Open_SET(u8 num,u8 status);
void Show_SET(u8 num,u8 status);
void Key_Scan(void);
void Key_Process(void);
#endif


