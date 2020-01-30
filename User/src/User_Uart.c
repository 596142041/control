#include "stm32f10x.h"
#include "User_Uart.h"
#include "Sys_delay.h"
////电池串口
u8 length_1, length_2, length_3;
u8 H_Length = 0;
u8 L_Length = 0;
u16 RX_received_count = 0;
//////接收缓存
u8 RxBuffer1[100];
u8 RxBuffer2[100];
u8 RxBuffer3[20];
u8 RxBuffer4[20];
u8 RxBuffer5[20];


////接收长度寄存器
u8 Uart_TX_Buff1[20];
u8 Uart_TX_Buff2[10];
u8 Uart_TX_Buff3[10];
u8 Uart_TX_Buff5[30];
u8 Bat_info[20];
u8 motion_info[20];
u8 niutou_info[20];
u8 RxCounter1 = 0;
u8 RxCounter2 = 0;
u8 RxCounter3 = 0;
u8 RxCounter4 = 0;
u8 RxCounter5 = 0;

u8 Uart_RX_Flag1;
u8 Uart_RX_Flag2;
u8 Uart_RX_Flag3;
u8 Uart_RX_Flag4;
u8 Uart_RX_Flag5;

u8 Set_num = 0;
u8 Low_set_num = 0;
u8 SixPump_Commnum = 0;
u8 SixPump_CommData1 = 0;
u8 SixPump_CommData2 = 0;

u8 Six_Handle_Flag = 0;
u8 Six_Handle_Flag_next = 0;

u8  SV_Control_num = 0;
u8  SV_Control_status = 0;

s32 step_2[3];
u16 accel_2[3];
u16 decel_2[3];
u16 speed_2[3];

u8 woshou_flag = 0;


void ModBusCRC16(u8 *cmd, int len)
{
    u16 i, j, tmp, CRC16;

    CRC16 = 0xFFFF;             //CRC??????
    for (i = 0; i < len; i++)
    {
        CRC16 ^= cmd[i];
        for (j = 0; j < 8; j++)
        {
            tmp = (u16)(CRC16 & 0x0001);
            CRC16 >>= 1;
            if (tmp == 1)
            {
                CRC16 ^= 0xA001;
            }
        }
    }
    cmd[i++] = (u8) (CRC16 & 0x00FF);
    cmd[i++] = (u8) ((CRC16 & 0xFF00) >> 8);
}


//串口1初始化
void UART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    //  USART_ClockInitTypeDef USART_ClockInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    //初始化串口1
    USART_Init(USART1, &USART_InitStructure);

    //使能串口1接收发送中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    //使能串口1
    USART_Cmd(USART1, ENABLE);

    /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    	如下语句解决第1个字节无法正确发送出去的问题 */
//	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送外城标志，Transmission Complete flag */

}


//串口1 IO口初始化  PA9--Tx  PA10--Rx
void UART1_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // Configure USART1_Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;               ////串口1  pin9  占用
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1_Rx as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;              ////串口1 pin10  占用
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
//串口1 发送一个字符
u8 Uart1_PutChar(u8 ch)
{
    // Write a character to the USART
    USART_SendData(USART1, ch);
    //  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    return ch;

}



void Uart_Send1(u8* buf, u8 len)
{
    u8 i;
    // USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//使能串口5接收、发送中断
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);                //关闭USART1的TXE中断
    for(i = 0; i < len; i++)
    {
        Uart1_PutChar(*buf++);
    }
    for(i = 0; i < len; i++)
        buf[i] = 0;
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);                //关闭USART1的TXE中断
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能串口5接收、发送中断
}





void UART2_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
// GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);           端口重映射


    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    //使能串口2接收
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART2, ENABLE);
}
//串口2 IO口初始化   PA2   PA3
void UART2_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure USART1_Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                       ////pin2脚 串口2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1_Rx as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                       ////pin3脚 串口2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
//串口2 发送一个字符
u8 Uart2_PutChar(u8 ch)
{
    // Write a character to the USART
    USART_SendData(USART2, ch);
    //  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
    }
    return ch;

}



void Uart_Send2(u8* buf, u8 len)
{
    u8 i;
    //  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
//    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    for(i = 0; i < len; i++)
    {
        Uart2_PutChar(*buf++);
    }
    for(i = 0; i < len; i++)
        buf[i] = 0;
    RxCounter2 = 0;
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);                //关闭USART1的TXE中断
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//使能串口5接收、发送中断

}

//串口3初始化
void UART3_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);

    //使能串口3接收
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);
}
//串口3 IO口初始化   PB10--tx  PB11--Rx   uart3
void UART3_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // Configure USART1_Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure USART1_Rx as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

u8 Uart3_PutChar(u8 ch)
{
    USART_SendData(USART3, ch);


    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
    {
    }
    return ch;
}


/*************************************************************
函数名称:Uart_Send
功    能:发送函数
参    数:P--数据指针  ；Length--数据长度
返 回 值:无
说    明:无
*************************************************************/
/*******UART1发送函数**************/
void Uart_Send3(u8* buf, u8 len)
{
    u8 i;
    for(i = 0; i < len; i++)
    {
        Uart3_PutChar(*buf++);
    }
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);                //关闭USART1的TXE中断

}



//串口3初始化
void UART5_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(UART5, &USART_InitStructure);

    //使能串口3接收
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    USART_Cmd(UART5, ENABLE);
}
//串口3 IO口初始化   PB10--tx  PB11--Rx   uart3
void UART5_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    // Configure USART1_Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // Configure USART1_Rx as input floating

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

u8 Uart5_PutChar(u8 ch)
{
    USART_SendData(UART5, ch);


    while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
    {
    }
    return ch;
}


/*************************************************************
函数名称:Uart_Send
功    能:发送函数
参    数:P--数据指针  ；Length--数据长度
返 回 值:无
说    明:无
*************************************************************/
/*******UART1发送函数**************/
void Uart_Send5(u8* buf, u8 len)
{
    u8 i;
    for(i = 0; i < len; i++)
    {
        Uart5_PutChar(*buf++);
    }
    USART_ITConfig(UART5, USART_IT_TXE, DISABLE);                //关闭USART1的TXE中断

}



void Uart1_Work(void)
{
    u8 i = 0;

    if(Uart_RX_Flag1)
    {
        Uart_RX_Flag1 = 0;
   //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//s失能串口5接收、发送中断

        Set_num = RxBuffer1[2];
        Low_set_num = RxBuffer1[3];
        //


        for(i = 0; i < 10; i++)
        {
            Uart_TX_Buff2[i] = 0;
            Uart_TX_Buff1[i] = 0;
        }
        for(i = 0; i < 20; i++)
            RxBuffer1[i] = 0;

      //  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能串口5接收、发送中断
    }


}


void Uart2_Work(void)
{

    if(Uart_RX_Flag2 == 1)
    {
        Uart_RX_Flag2 = 0;
    }

}



void Uart3_Work(void)
{
    if(Uart_RX_Flag3)
    {
        Uart_RX_Flag3 = 0;
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//s失能串口5接收、发送中断
        Uart_Send3(RxBuffer3, 9);

        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能串口5接收、发送中断
    }

}



void UART_All_Init(u8 uart_num)
{
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);    //串口要开启复用时钟，不然会造成串口发送数据失败
    switch(uart_num)
    {
    case 0x01: {
        UART1_GPIO_Configuration();
        UART1_Configuration();
    }
    break;
    case 0x02: {
        UART2_GPIO_Configuration();
        UART2_Configuration();
    }
    break;
    case 0x03: {
        UART3_GPIO_Configuration();
        UART3_Configuration();
    }
    break;
    case 0x04: {
        UART3_GPIO_Configuration();
        UART3_Configuration();
    }
    break;
    case 0x05: {
        UART5_GPIO_Configuration();
        UART5_Configuration();
    }
    break;
    default :
        break;



    }
//  UART4_GPIO_Configuration();
//  UART4_Configuration();
//  UART5_GPIO_Configuration();
//  UART5_Configuration();

}

/*
//函 数 名：AscToHex()
//功能描述：把ASCII转换为16进制
unsigned char AscToHex(unsigned char aHex)
{
  if((aHex>=0)&&(aHex<=9))
  aHex += 0x30;
  else if((aHex>=10)&&(aHex<=15))//A-F
  aHex += 0x37;
  else aHex = 0xff;
  return aHex;
}
//函 数 名：HexToAsc()
//功能描述：把16进制转换为ASCII
unsigned char HexToAsc(unsigned char aChar){
if((aChar>=0x30)&&(aChar<=0x39))
aChar -= 0x30;
else if((aChar>=0x41)&&(aChar<=0x46))//大写字母
aChar -= 0x37;
else if((aChar>=0x61)&&(aChar<=0x66))//小写字母
aChar -= 0x57;
else aChar = 0xff;
return aChar;
}
*/

////////////////////////////////////////////////////////////////
//功能:		将多字节16进制数转换为ASCII字符
//入口:		x:待转换16进制数,x>=0且x<=F
//出口:		返回转换完成后的ASCII码字符数据,返回0为失败
////////////////////////////////////////////////////////////////
u8 GetASCII(u8 x)
{
    if(x > 0x0f)
    {
        return 0;
    }

    if(x > 9)
    {
        return ('A' + x - 10);
    }
    return ('0' + x);
}
/*
for(i=0;i<NUMBER;i++)
	{
		BufferOut[i*2] = GetASCII(BufferIn[i]/0x10);
		BufferOut[i*2+1] = GetASCII(BufferIn[i]%0x10);
	}
*/
u8 HexToChar(u8 bChar)
{
    if(((bChar > 0x30) && (bChar < 0x39)) || (0x30 == bChar) || (0x39 == bChar))
    {
        bChar -= 0x30;
    }
    else if(((bChar > 0x41) && (bChar < 0x46)) || (0x41 == bChar) || (0x46 == bChar))
    {
        bChar -= 0x37;
    }
    else if(((bChar > 0x61) && (bChar < 0x66)) || (0x61 == bChar) || (0x66 == bChar))
    {
        bChar -= 0x57;
    }
    else
    {
        bChar = 0xff;
    }
    return bChar;
}

u8 Hex2Char(u8 bchar1, u8 bchar2)
{
    u8 H_char, L_char;
    u8 hex_temp;
    H_char = HexToChar(bchar1);
    L_char = HexToChar(bchar2);
    hex_temp = (((H_char << 4) & 0xF0) | (L_char & 0x0F));
    return hex_temp;
}

void Pack_Send(u8 CMD, u16 length, u8 *Info_buff)
{
    u8 i = 0;
    // u16 length_temp;
    u8 addr_b = 0x00;
    u16 length_temp;
    u8 length1;
    u8 length2;
    u8 length3;
    u16 length_sum;
    u16 length_id;
    u8 length_id1;
    u8 length_id2;
//  u8 length_id3;
    u8 Send_temp[30];
    u16 checksum;
    u8 checksum1, checksum2, checksum3, checksum4;

    Send_temp[0] = 0x7E;
    Send_temp[1] = 0x30; //版本号1
    Send_temp[2] = 0x31; //版本号2

    Send_temp[3] = GetASCII(addr_b / 0x10); //地址1
    Send_temp[4] = GetASCII(addr_b % 0x10); //地址2

    Send_temp[5] = GetASCII(0x46 / 0x10); //CD1-1
    Send_temp[6] = GetASCII(0x46 % 0x10); //CD1-2

    Send_temp[7] = GetASCII(CMD / 0x10); //CD2-1
    Send_temp[8] = GetASCII(CMD % 0x10); //CD2-2
//  Send_temp[9]=0x30;  //LENGTH-1
//  Send_temp[10]=0x31;  //LENGTH-2
//  Send_temp[11]=0x30;  //LENGTH-3
//  Send_temp[12]=0x31;  //LENGTH-4
    length_temp = length * 2;
    length1 = length_temp & 0x000F;
    length2 = (length_temp & 0x00F0) >> 4;
    length3 = (length_temp & 0x0F00) >> 8;
    length_sum = length3 + length2 + length1;
    length_sum = (((~(length_sum % 0x10)) + 0x01) & 0x000F) << 12;
    length_id = length_sum | (length & 0x0FFF);
    length_id1 = (length_id & 0xFF00) >> 8;
    length_id2 = (length_id & 0x00FF);
    // length_id3=(length_id&0x000F);
//  length_id2=(length_id&0x00FF);



    Send_temp[9] = GetASCII(length_id1 / 0x10); //LENGTH-1
    Send_temp[10] = GetASCII(length_id1 % 0x10); //LENGTH-2

    Send_temp[11] = GetASCII(length_id2 / 0x10); //LENGTH-3
    Send_temp[12] = GetASCII(length_id2 % 0x10); //LENGTH-4


    for(i = 0; i < length; i++)
    {
        Send_temp[13 + i * 2] = Info_buff[i] / 0x10;
        Send_temp[13 + i * 2 + 1] = Info_buff[i] % 0x10;
    }


    for(i = 1; i < 12 + length * 2; i++)
    {
        checksum += Send_temp[i];
    }
    checksum = (~(checksum % 0xFFFF)) + 0x01;
    checksum1 = (checksum & 0xF000) >> 12;
    checksum2 = (checksum & 0x0F00) >> 8;
    checksum3 = (checksum & 0x00F0) >> 4;
    checksum4 = (checksum & 0x000F);

    Send_temp[13 + length * 2] = GetASCII(checksum1 / 0x10); //Checksum-1
    Send_temp[13 + length * 2 + 1] = GetASCII(checksum1 % 0x10); //Checksum-2
    Send_temp[14 + length * 2] = GetASCII(checksum2 / 0x10); //Checksum-3
    Send_temp[14 + length * 2 + 1] = GetASCII(checksum2 % 0x10); //Checksum-4
    Send_temp[15 + length * 2] = GetASCII(checksum3 / 0x10); //Checksum-5
    Send_temp[15 + length * 2 + 1] = GetASCII(checksum3 % 0x10); //Checksum-6
    Send_temp[16 + length * 2] = GetASCII(checksum4 / 0x10); //Checksum-7
    Send_temp[16 + length * 2 + 1] = GetASCII(checksum4 % 0x10); //Checksum-8
    Send_temp[16 + length * 2 + 2] = 0x0D;

}


