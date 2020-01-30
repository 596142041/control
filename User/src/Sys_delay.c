#include "Sys_delay.h"


static __IO uint32_t TimingDelay;
u16 mcu_beep_time_flag = 0;

u32  Counter_100uS = 0;
u32  Counter_1mS = 0;
u32  Counter_10mS = 0;
u32  Counter_100mS = 0;
u32  Flag_1mS = 0;
u32  Flag_10mS = 0;
u32  Flag_100mS = 0;
u32  Flag_1S = 0;
u32  Flag_fs = 0;
u16 system_start_flag = 0;
void SysTick_Init(void)
{
    /* SystemCoreClock / 10     100ms中断一次
     * SystemCoreClock / 1000     1ms中断一次
     * SystemCoreClock / 100000  10us中断一次
     * SystemCoreClock / 1000000  1us中断一次
     */
    if(SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1);
    }
}

void Sys_Delay_1ms(__IO u32 nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);//在延时过程中无限等待
}

void Sys_Delay_1s(__IO u32 nTime)
{
    TimingDelay = nTime * 1000;

    while(TimingDelay != 0);//在延时过程中无限等待
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    {
        TimingDelay--;
    }
}


void delay_us(u32 time)
{
    u32 i = 8 * time;
    while(i--);
}
void delay_ms(u32 time)
{
    u32 i = 8000 * time;
    while(i--);
}


