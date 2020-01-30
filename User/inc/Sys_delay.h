#ifndef _SYS_DELAY_H_
#define _SYS_DELAY_H_
#include "stm32f10x.h"

#define Bitset(var,bitno)   ((var)|=1<<(bitno)) //Î»²Ù×÷
extern u16 mcu_beep_time_flag;

extern  u16 system_start_flag;
extern u32  Counter_10uS;
extern u32  Counter_100uS;
extern u32  Counter_1mS;
extern u32  Counter_10mS;
extern u32  Counter_100mS;

extern u32 Flag_1mS;
extern u32 Flag_10mS;
extern u32 Flag_100mS;
extern u32 Flag_1S; 
extern u32 Flag_fs; 

void SysTick_Init(void);
extern void Sys_Delay_1ms(__IO u32 nTime);
extern void Sys_Delay_1s(__IO u32 nTime);
extern void TimingDelay_Decrement(void);

extern void delay_us(u32 time);
extern void delay_ms(u32 time);

#endif //

