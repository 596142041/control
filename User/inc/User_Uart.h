#ifndef _UART_H
        #define _UART_H

#define CID1          0x46
#define CID2_Analog   0x42
#define CID2_Warning  0x44
#define CID2_num      0x90
#define CID2_Version  0xC1
#define CID2_Value    0xB1
/*
typedef struct 
{
	u8  SOI;		//
        u8  VER;
	u8  ADR;		//
	u8  CID1;		//
        u8  CID2;
	u8  Length[2];		//
        u8 *INFO;
        u8 CHKSUM[2];
        u8 EOI;
}UART_Com;

extern UART_Com BatteryCom;
*/
extern u8 RxBuffer1[100];
extern u8 RxBuffer2[100];
extern u8 RxBuffer3[20];
extern u8 RxBuffer4[20];
extern u8 RxBuffer5[20];


////接收长度寄存器



extern u8 RxCounter1;
extern u8 RxCounter2;
extern u8 RxCounter3;
extern u8 RxCounter4;
extern u8 RxCounter5;


////接收最大长度限制
#define UART_RX_MAX1 100
#define UART_RX_MAX2 100
#define UART_RX_MAX3 100
#define UART_RX_MAX4 100
#define UART_RX_MAX5 100

extern u8 Uart_RX_Flag1;
extern u8 Uart_RX_Flag2;
extern u8 Uart_RX_Flag3;
extern u8 Uart_RX_Flag4;
extern u8 Uart_RX_Flag5;

extern u8 Uart_TX_Buff1[20];
extern u8 Uart_TX_Buff5[30];
extern u8 niutou_info[20];
extern u8 Bat_info[20];
extern u8 motion_info[20];

void UART1_Configuration(void);
void UART1_GPIO_Configuration(void);
void Uart_Send1(u8* buf , u8 len);

void UART2_Configuration(void);
void UART2_GPIO_Configuration(void);
void Uart_Send2(u8* buf , u8 len);

void UART3_Configuration(void);
void UART3_GPIO_Configuration(void);
void Uart_Send3(u8* buf , u8 len);

void UART4_Configuration(void);   
void UART4_GPIO_Configuration(void);
void Uart_Send4(u8* buf , u8 len);

void UART5_Configuration(void);
void UART5_GPIO_Configuration(void);
void Uart_Send5(u8* buf , u8 len);

void Uart1_Work(void);
void Uart2_Work(void);
void Uart3_Work(void);
void Uart4_Work(void);
void Uart5_Work(void);

void UART_All_Init(u8 uart_num);


extern  u8 length_1,length_2,length_3;
extern  u8 H_Length;
extern  u8 L_Length;
extern  u16 RX_received_count;

u8 GetASCII(u8 x);
u8 HexToChar(u8 bChar);
extern u8 Hex2Char(u8 bchar1, u8 bchar2); 
void Pack_Send(u8 CMD,u16 length,u8 *Info_buff);
void ModBusCRC16(u8 *cmd, int len);
#endif