#ifndef __STATE_H__
#define __STATE_H__
#include "stm32f10x.h"
//该文件内定义各种输入和输出状态
typedef union _Input_state
{
	u16 all;
	struct
	{
		u16	yaoshi:1;//钥匙
		u16 PC_Fb:1;//PC反馈
		u16 JT:1;//急停
		u16 revs:13;//保留位
	}bits;
}Input_state;
typedef struct  
{
	int period;//
	int cnt;//
	char time_out_flag;
	int run;
	int time_out;
	char update_flag;//
}timer;
#endif