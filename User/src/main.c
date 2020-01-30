
#include "stm32f10x.h"
#include <stdio.h>
#include "Comm_IO.h"
#include "User_Uart.h"
#include "GYTH_CAN.h"
#include "sys_delay.h"
#include "SPI_show.h"
#include "state.h"
void NVIC_Configuration(void);
/**
* @brief  Main progra·m.
* @param  None
* @retval None
*/

//u8 can_send_buff[8] = {1, 2, 3, 4, 5, 6, 7, 0Xff};
u8 status_alarm = 0;
u8 computer_send_flag;
u8 Bg_flag=0;
float ADC0_Value;//
float ADC1_Value;//
u16 *ABC;
u16 abd[2];
u16 EE=1;
u8 bc=0;

u8 state[8];
u8 bg[8];
u8 eero[8];
u8 pb=0;
u8 Open_PC_FB_status_flag=0;
u8 gj;
u8 gj1;
u8 gj2;
u8 ov;
u8 oi;
u8 ow;
u8 ph;
u8 cmd0;
u16 P_V;
u8 yaoshi=0;
u8 lw0;
u8 lw1;
u16 dj0;
u8  rw0;
u8  rw1;
u16 dj1;
u16 N_V0;  //负端电压
u16 P_V0;  //正端电压
u8 Uartflag=0;
u8 old_s=0;

timer sz1,beep_timer;
//  int out_PID=0;
short int in_turn = 0;
u16 in_adc, out_adc;
u8 Power_switch_flag = 0;
u8 source=0;//卡扣标示
u16 motion_input_status = 0;
u8  motion_input_buffer[12] = {0};
u8 motion_info_temp[20] = {0};
 u8 motion_info_temp1[20] = {0};
__INLINE  void init_dog(void)
{
	IWDG_WriteAccessCmd(0x5555);
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	IWDG_SetReload(0x0FFF);
	IWDG_WriteAccessCmd(0xaaaa);
}
__INLINE  void feed_dog(void)
{
	IWDG_ReloadCounter();
}
__INLINE void enable_dog(void)
{
		IWDG_Enable();
}


int  main(void)
{
	u8 i = 0;
	sz1.period = 2000;
	sz1.run = 0;
	sz1.time_out = 5000;
	sz1.update_flag = 0;//表示定时器达到
	sz1.time_out_flag = 0;
	beep_timer.period = 800;
	beep_timer.run  = 0;
	beep_timer.update_flag = 0;
  //  Off_Power_SET(0);   程序运行不作判断,原始低电位,不作处理
    MCU_BEEP_Config();
    SysTick_Init();
    Turn_LED_Init();   //调试LED管脚初始化
    COM_GPIO_Init();
    Expose_GPIO_Init();
    UART_All_Init(1);     //串口初始化  //发送给电量显示板电量信息
    UART_All_Init(2);    //与工控机相连，，接收和发送消息命令
    CAN_GPIO_Configuration();  //Can通信初始化
    CAN_Configuration();     //Can 通信初始化
    GPIO_Display_Init();
    NVIC_Configuration();
    Expose_SET(0, 0);//软件控制一档信号清零
    Expose_SET(1, 0);//软件控制二档信号清零
//  YK1_time_flag=2000;
//  Expose_time_flag=7000;
    
	
	    Show_SET(3, 1);   //蓝灯灭
      Show_SET(2, 1);   //绿灯灭
      Show_SET(1, 1);   //红灯亮
    MCU_BEEP_SET(1);
    delay_ms(1000);

    Show_SET(3, 0);   //蓝灯灭
    Show_SET(2, 0);   //绿灯灭
     Show_SET(1, 0);   //红灯亮
    Expose_SET(0, 0);  //软件控制一档信号清零////与老版本改进!!
    MCU_BEEP_SET(0);
		
		
		init_dog();
		enable_dog();		
    while (1)
    {
		feed_dog();
        if(0 == Power_KE_FB_status)     //如果钥匙开关打开
        {  	
			Open_SET(8, 1);	
            yaoshi=1;  //钥匙开关打开
                if(Open_PC_FB_status == 1)// if(Open_PC_FB_status == 1)
                {    
					EE=0;
					Open_PC_FB_status_flag=1;
					Open_SET(1, 1);   //路由器，工控机电源
					old_s=1;  //原始状态位
                    if((1 == SZ1_flag) || (1 == YK1_flag)) ///////曝光一阶段，手闸一档，或者遥控一档信号
                    {     
							SZ1_time_flag0=1; 
							mcu_beep_time_flag=0;
                        if(0 == YK1_count_flag)//&&(1 == YK1_flag))//手闸曝光1档zxc20190812增加1 == YK1_flag判断
                        {   

                            YK1_count_flag = 1;
                            //Expose_SET(0, 1);  //MCU软件一档信号发出  更改
                            YK1_time_flag = 1000; //延时给高压同步盒子信号1s
                            BG_ZS_Flag = 0;  //修改

                            BG_ZS_Flag = 1;
                            time_flag1 = 9000;  //按下手闸开始计数，9S内必须完成整个曝光环节
                            if(1 == Power_CE_FB_status) //如果交流接通
                               Expose_SET(4, 1);  //断开交流接触器
                            Show_SET(3, 1);   //蓝灯亮
                            Show_SET(2, 1);   //绿灯亮
                            Show_SET(0, 0);   //蜂鸣器响起
                            mcu_beep_time_flag = 1200;
                        }
                        if((0 == YK1_time_flag) && (1 == YK1_count_flag) && (1 == YK1_flag)) //只有在遥控手闸下才进行延时发信号
                        {
                            Open_SET(3, 1); //给高压同步盒的信号
                              YK2_flag=1; //修改
                            YK1_count_flag = 0;

                        }


                        if((1 == SZ2_flag) || (1 == Open_JN_FB_status)) //二阶段，，手闸二挡,或者佳能高压同步盒子反馈了同步信号
                        { 
							if(beep_timer.update_flag == 0)
							{
								MCU_BEEP_SET(1);
							}
							else
							{
								MCU_BEEP_SET(0);
							}					
                            if( 1 == BG_ZS_Flag)
                            {
                                BG_ZS_Flag = 2;
                                mcu_beep_time_flag = 6000;
								                MCU_BEEP_SET(0);
                            }
								Show_SET(0, 0);   //蜂鸣器灭掉
								//Expose_SET(1,0);   //二档信号发出  //修改
								Show_SET(3, 0);   //蓝灯灭
								Show_SET(2, 1);   //绿灯灭
								Show_SET(1, 1);   //红灯亮
								//定时器开启，延时，关灯，关交流接触器
								Expose_time_flag = 8000;
									
							}
							
							
								
							
                        }
                    //}
                    if(((1 == BG_finish_flag) || (0 == Expose_time_flag) || (0 == time_flag1)) && ((0 == SZ2_flag) && (0 == SZ1_flag) && (0 == YK2_flag) && (0 == YK1_flag))) //收到曝光信号后将个信号断开
                    {//
                       { MCU_BEEP_SET(0);//关闭蜂鸣器
												 Expose_SET(1,0);
												 
										    }
                        if(BG_finish_flag == 1)//曝光反馈信号上升沿触发中断BG_FB，表示高压曝光成功
						{
							MCU_BEEP_SET(0);
							BG_finish_flag = 0;//清中断信号
						}
                        if( BG_ZS_Flag == 2)
                        {
							             if(Power_switch_flag == 2)  //如果在充电，则恢复充电状态，后期通过充电状态来进行控制
                            {
                                //   BG_finish_flag=0;
                                BG_ZS_Flag = 0;
                                Expose_SET(4, 0);  //断开交流接触器
                                Expose_SET(0,0);   //一档信号回收
                                Expose_SET(1,0);   //二档信号回收
                                Show_SET(3, 0);   //蓝灯灭
                                Show_SET(2, 0);   //绿灯灭
                                Show_SET(1, 0);   //红灯亮
                              
                                Open_SET(3, 0);
                            }
                        }
                        else if(BG_ZS_Flag == 1)   //二挡信号未发出，或者高压同步盒信号未发出，则取消曝光,发出警告信息
                        {
                            BG_ZS_Flag = 0;
                            Expose_SET(4, 0);  //断开交流接触器
                            Expose_SET(0,0);   //一档信号回收
                            Expose_SET(1,0);   //二档信号回收
                            Show_SET(3, 0);   //蓝灯灭
                            Show_SET(2, 0);   //绿灯灭
                            Show_SET(1, 0);   //红灯亮
                            Open_SET(3, 0);

                        }
                        else if(BG_ZS_Flag == 0)
                        {
                            Expose_SET(4, 0);  //断开交流接触器
                            Expose_SET(0,0);   //一档信号回收
                            Expose_SET(1,0);   //二档信号回收
                            Show_SET(3, 0);   //蓝灯灭
                            Show_SET(2, 0);   //绿灯灭
                            Show_SET(1, 0);   //红灯亮
                            Open_SET(3, 0);
                        }

                    }
                }
                else if(Open_PC_FB_status == 0) // && (open_time == 0))//工控机未开机情况下,关机断设备电源
                {
                 
									 if (EE==0)               ///判断标示位,关掉工控电源
									 { Power_SET(2, 0);
										  Open_SET(1, 0);
										  EE=1;
										  old_s=0;
									  } 
                    if((SZ2_flag == 1) || (SZ1_flag == 1) || (YK1_flag == 1) || (YK2_flag == 1))
                    {
                        SZ1_flag = 0;
                        SZ2_flag = 0;
                        YK1_flag = 0;
                        YK2_flag = 0;
                    }
										
                }


            

            //检测是否急停
            if(0 == Power_ST_FB_status)//未按下急停则打开电机电源48V
            {   bc=0;
                Power_SET(3, 1);//急停开管关闭后,关断电源,电磁刹车也关闭了!!!!
							  LED_SPI_Send1(0,0);
							  can_send_buff[0]=0;
							  can_send_buff[1]=0;
							  can_send_buff[2]=0;
							  can_send_buff[3]=0;
							  can_send_buff[4]=0;
							  can_send_buff[5]=0;
							  can_send_buff[6]=0;
							  can_send_buff[7]=0;
                CAN_TX(can_send_buff,0x51);	//发送标示位

            }
            else if(1 == Power_ST_FB_status)
            {   bc=1;
                LED_SPI_Send1(7,2);
								 Power_SET(0, 0);  //高压开关电源开关
								 Power_SET(1, 0);   //限速器关闭
							   Show_SET(4, 0);   //高压开
							  can_send_buff[0]=1;
							  can_send_buff[1]=1;
							  can_send_buff[2]=1;
							  can_send_buff[3]=1;
							  can_send_buff[4]=1;
							  can_send_buff[5]=1;
							  can_send_buff[6]=1;
							  can_send_buff[7]=1;
							  CAN_TX(can_send_buff,0x51);	//发送标示位
            }
					}  //添加
		else if(1 == Power_KE_FB_status) //如果要是开关关闭
        {     yaoshi=0;
              Power_SET(3,0);                //
               delay_ms(1);
        }

			 ///CAN通信处理 接收到牛头数据转发
        if(ret == 0x01)
        {
            ret = 0;
            niutou_info[0]= 0x5A;
            niutou_info[1]= 7;
            niutou_info[2]= 0x04;
            niutou_info[3]= RXCAN_Protol2[0];
            niutou_info[4]= RXCAN_Protol2[1];
            niutou_info[5]= RXCAN_Protol2[2];
            niutou_info[6]= RXCAN_Protol2[3];
            niutou_info[7]= RXCAN_Protol2[4];
            ModBusCRC16(&niutou_info[1], 7);
            niutou_info[10]= 0xA5;
            Uart_Send2(niutou_info, 11); 
        }
        else if(ret == 0x02)//接受到充电数据
        {
            ret = 0;
            
                ///////////////////////////第一步，暍送电量到电量显示板
                Uart_TX_Buff1[0] = 0x5A;
                Uart_TX_Buff1[1] = 11; //length
                Uart_TX_Buff1[2] = RXCAN_Protol[0];
					                     gj=RXCAN_Protol[1];//高8位
                Uart_TX_Buff1[3] = RXCAN_Protol[1];
                Uart_TX_Buff1[4] = RXCAN_Protol[2];
					                    gj1=RXCAN_Protol[2]; //低8位
                Uart_TX_Buff1[5] = RXCAN_Protol[3];
                Uart_TX_Buff1[6] = RXCAN_Protol[4];
                Uart_TX_Buff1[7] = RXCAN_Protol[5];
                Uart_TX_Buff1[8] = RXCAN_Protol[6];
                Uart_TX_Buff1[9] = bc;
                Uart_TX_Buff1[10] = 0xA5; //
                Uart_Send1(Uart_TX_Buff1, 11);//串口1暍送数据到电量显示板
                ///////////////////////////////第二步，将电量信息转存到工控机
                Bat_info[0] = 0x5A;
                Bat_info[1] = 10;
                Bat_info[2] = 0x02;
                Bat_info[3] = RXCAN_Protol[0];
                Bat_info[4] = RXCAN_Protol[1];   //电压高8位数据P_V         Bat_info[4]
                Bat_info[5] = RXCAN_Protol[2];   //电压低8位数据            Bat_info[5]
                Bat_info[6] = RXCAN_Protol[3];     //电压高8位数据N_V         Bat_info[6]
                Bat_info[7] = RXCAN_Protol[4];   //电压低8位数据            Bat_info[7]
                Bat_info[8] = RXCAN_Protol[5];    //电流高8位
                Bat_info[9] = RXCAN_Protol[6];    //电流低8位
                Bat_info[10] = RXCAN_Protol[7];
							           gj2=RXCAN_Protol[7]>>7;   //修改市电
												 ov=(RXCAN_Protol[7]>>6);   //修改过压
												 oi=(RXCAN_Protol[7]>>5);   //修改过流
												 ow=(RXCAN_Protol[7]>>4);   //修改过温
												 ph=(RXCAN_Protol[7]>>3);   //修改平衡
												 cmd0=(RXCAN_Protol[7]>>2);  //修改充满
                ModBusCRC16(&Bat_info[1], 10);
                Bat_info[13] = 0xA5; //               
                Uart_Send2(&Bat_info[0],14);//串口2暍送到工控机
								if(ph==1)
								{ LED_SPI_Send1(5,3); //错误码53
								} else if ( ph==0)
							 {LED_SPI_Send1(0,0);		                    
							 }
									if(ow==1)
								{ LED_SPI_Send1(5,4); //错误码53
								} else if ( ow==0)
							 {LED_SPI_Send1(0,0);		                    
							 }
									if(oi==1)
								{ LED_SPI_Send1(5,6); //错误码53
								} else if ( oi==0)
							 {LED_SPI_Send1(0,0);		                    
							 }
									if(ov==1)
								{ LED_SPI_Send1(5,7); //错误码53
								} else if ( ov==0)
							 {LED_SPI_Send1(0,0);		                    
							 }
									//电池报警信息
								 P_V=(gj<<8)|gj1;
								if((P_V<=1280)&&(gj2==0))        ////只有在接受到充电数据的时候,才开始判断是否关机
								//    {  // GPIO_SetBits(GPIOA, GPIO_Pin_6);
									  {// dlg=10; 关闭控制位
											GPIO_SetBits(GPIOA, GPIO_Pin_6);
                      // MCU_BEEP_SET(1);
										}
								//Off_Power_SET(1);
							//	    }
							//	else
							//     	{  Off_Power_SET(0);   //判断电压以外的范围为低电位
							//	    }
								if((P_V<=1200)&&(P_V>=1160)&&(gj2==0))
								{   delay_ms(1000); 
									  MCU_BEEP_SET(1);
									  delay_ms(1000);
									  MCU_BEEP_SET(0);    ////补测电压情况
									  LED_SPI_Send1(5,0); //错误码50
									
								}else if(P_V<1160)
								  {  MCU_BEEP_SET(0);
										 LED_SPI_Send1(0,0); 
									}
								////电池11V关闭所有电源
						
								
							 if(0 == SZ2_flag)
								 {
									 if((P_V<=1150)&&(gj2==0))  
								   {  delay_ms(5000);
										 Power_SET(2, 0);
										  Open_SET(1, 0);
										  MCU_BEEP_SET(0); 
										  Power_SET(0, 0);  //高压开关电源开关
										  Power_SET(1, 0);
										  Show_SET(4, 0);  
										  LED_SPI_Send1(5,1); //错误码51
								   }
								 } else
								  {   //delay_ms(5000);
										 if((P_V<=1150)&&(gj2==0))  
								   {     Power_SET(2, 1);
										  Open_SET(1, 1);
										
										  Power_SET(0, 1);  //
										  Power_SET(1, 1);
										  Show_SET(4, 1);  //高压使能
								   }
								  }
										  
									
								
                //解析充电板的错误信息
                if( Bat_info[10] >> 6 == 1)
                    flash_value = 0x03;
                if( Bat_info[10] >> 5 == 1)
                    flash_value = 0x04;
                if( Bat_info[10] >> 4 == 1)
                    flash_value = 0x05;
                if( Bat_info[10] >> 3 == 1)
                    flash_value = 0x06;

            
        }
        else if((ret == 0x03)) 
        {
            u8 motion_info_temp[20] = {0};
            ret = 0;

            motion_info[0] = RXCAN_Protol1[0] ; //////端口输入高8位    
            motion_info[1] = RXCAN_Protol1[1] ; //////端口输入低8位
            motion_input_status = (motion_info[0] << 8) | motion_info[1];  
            for(i = 0; i < 12; i++)
            {
                motion_input_buffer[11 - i] = (motion_input_status >> i) & 0x0001;
            }
       
            motion_info[2] = RXCAN_Protol1[2] ; //////左电机温度高8
						            lw0= RXCAN_Protol1[2];
            motion_info[3] = RXCAN_Protol1[3] ; //////左电机低8位
						            lw1= RXCAN_Protol1[3];
            motion_info[4] = RXCAN_Protol1[4] ; //////右电机温度高8
						            rw0=RXCAN_Protol1[4] ;
            motion_info[5] = RXCAN_Protol1[5] ; //////右电机低8位
						            rw1=RXCAN_Protol1[5] ;
                        dj0=lw0<<8|lw1;       ////电机温度读取
						            dj1=rw0<<8|rw1;       ////电机温度读取
            if((motion_info[3] > 70) || (motion_info[5] > 70))
            {
                flash_value = 0x02;
            }

            if(RXCAN_Protol1[0] == 0x03) //运动版驱动器报警信息
            {
                motion_info[6] = RXCAN_Protol1[6] ; //////电机驱动器报警信息
                if(motion_info[6] != 0)
                    flash_value = 0x01;
            }
            motion_info_temp[0] = 0x5A;
            motion_info_temp[1] = 10;
            motion_info_temp[2] = 3;//板号为03运动版
            motion_info_temp[3] = motion_info[0];   ////运动板状态信息 高8位
            motion_info_temp[4] = motion_info[1];   ////运动板状态信息 低8位
            motion_info_temp[5] = motion_info[2];
            motion_info_temp[6] = motion_info[3];
            motion_info_temp[7] = motion_info[4];
            motion_info_temp[8] = motion_info[5];
            motion_info_temp[9] = motion_info[6];
            motion_info_temp[10] = motion_info[7];
            ModBusCRC16(&motion_info_temp[1], 10);    //校验位
            motion_info_temp[13]=0xA5;
            Uart_Send2(&motion_info_temp[0], 14);//串口2暍送到PC
        }
				else if((ret == 0x04)) 
        {
            u8 motion_info_temp1[20] = {0};
            ret = 0;
						
            motion_info_temp1[0] = 0x5A;
            motion_info_temp1[1] = 10;
            motion_info_temp1[2] = 8;
            motion_info_temp1[3] = RXCAN_Protol11[0];
            motion_info_temp1[4] = RXCAN_Protol11[1];
            motion_info_temp1[5] = RXCAN_Protol11[2];
            motion_info_temp1[6] = RXCAN_Protol11[3];
            motion_info_temp1[7] = RXCAN_Protol11[4];
            motion_info_temp1[8] = RXCAN_Protol11[5];
            motion_info_temp1[9] = RXCAN_Protol11[6];
            motion_info_temp1[10] = RXCAN_Protol11[7];
						ModBusCRC16(& motion_info_temp1[1], 10);
           // ModBusCRC16(&motion_info_temp1[1], 10);  
            motion_info_temp[13]=0xA5;
            Uart_Send2(&motion_info_temp1[0], 14);
        }
					else if((ret == 0x06)) 
				{       ret=0;
					     source=RXCAN_Protol12[0];
					     if ( source==1)
							 {Open_SET(5, 1);
								// LED0=1;
								
								   if(Open_PC_FB_status==1)
									{	
										  Power_SET(0, 1);  //高压开关电源开关
										  Power_SET(1, 1);
										  Show_SET(4, 1); 
								      LED_SPI_Send1(6,7);   ////数字管显示状态
										 
									}
								 if(Open_PC_FB_status==0)
									{Power_SET(0, 0);  //高压开关电源开关
										  Power_SET(1, 0);
										  Show_SET(4, 0); 
								      LED_SPI_Send1(6,7);
										
									}
							 }
					      else if( ( source!=1)|(Open_PC_FB_status==0))
							 { Open_SET(5, 0);
								 LED_SPI_Send1(0,0);
									 Power_SET(0, 0);  //高压开关电源开关
										  Power_SET(1, 0);
										  Show_SET(4, 0); 
								   LED_SPI_Send1(0,0);   ////数字管显示状态
								 }
								 
							 
				       }
					
				
				}
		
			
				
	 if(computer_send_flag==1)//将接收到的PC数据直接发送到CAN总线上，不做任何处理
        {
           computer_send_flag=0; //目前(RxBuffer2[2]==0x04)cmd=04代表需要转发送给牛头板
//					if(RxBuffer2[2]==0x04)
//					{
//           can_send_buff[0]=RxBuffer2[3];//地址高
//           can_send_buff[1]=RxBuffer2[4];//地址低
//           can_send_buff[2]=RxBuffer2[5];//数据高
//           can_send_buff[3]=RxBuffer2[6];//数据低
//           can_send_buff[4]=RxBuffer2[7];//CRC_L
//           can_send_buff[5]=RxBuffer2[8];//CRC_H
//           can_send_buff[6]=RxBuffer2[9];//A5帧尾
//						can_send_buff[7]=0;
////           can_send_buff[7]=RxBuffer2[10];
//           CAN_TX(can_send_buff ,0x14);//0x14牛头地址，默认数据发送到牛头
//					}
					if(RxBuffer2[2]==0x05)//目前(RxBuffer2[2]==0x05)cmd=05设置延时曝光参数
					//  if(Bg_flag==1)
					{ Bg_flag=0;
						MCU_BEEP_SET(1);
						Expose_SET(0, 1);  //MCU软件一档信号发出
						feed_dog();
						delay_ms(2000);
						feed_dog();
						Expose_SET(1, 1);  //MCU软件二档信号发出
						delay_ms(2000);
						feed_dog();
						MCU_BEEP_SET(0);
						Expose_SET(0, 0);  //MCU软件二档信号发出
						Expose_SET(1, 0);  //MCU软件二档信号发出
						RxBuffer2[2]=0;
					}
//					if(RxBuffer2[2]==0x06)//目前(RxBuffer2[2]==0x06)cmd=06强制调教应变片
//					{
//           can_send_buff[0]=RxBuffer2[3];
//           can_send_buff[1]=RxBuffer2[4];
//           can_send_buff[2]=RxBuffer2[5];
//           can_send_buff[3]=RxBuffer2[6];
//           can_send_buff[4]=RxBuffer2[7];
//           can_send_buff[5]=RxBuffer2[8];
//           can_send_buff[6]=RxBuffer2[9];
//           can_send_buff[7]=RxBuffer2[10];
//           CAN_TX(can_send_buff ,0x16);//0x16牛头地址，默认数据发送到牛头
//					}
        }
     
///////////////////////////串口接收数据函数
				
        
//  Disp_battery(P_V);
	         // static u8 Uartflag=0;
            if (Uart_RX_Flag1)
              {
               Uart_RX_Flag1 = 0;
		           Uartflag=1;
              // timeout_uart = 500; //
               // status_charge = RxBuffer1[2];
							//	can_send_buff[3]=RxBuffer1[3];
							//	can_send_buff[4]=RxBuffer1[4];
							//	can_send_buff[5]=RxBuffer1[5];
							//	can_send_buff[6]=RxBuffer1[6];
             //  N_V0 = (RxBuffer1[3] << 8) | RxBuffer1[4];
             //  P_V0 = (RxBuffer1[5] << 8) | RxBuffer1[6];
            //   Charge_I = (RxBuffer1[7] << 8) | RxBuffer1[8];
           //    bc = RxBuffer1[9];
								
               USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
								CAN_TX(can_send_buff,0x71); //发送去充电板设置
               }
      
}


void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

