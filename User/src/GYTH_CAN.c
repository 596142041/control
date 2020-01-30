
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "GYTH_CAN.h"


/* Private define ------------------------------------------------------------*/
//
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


CanTxMsg TxMessage;
CanRxMsg RxMessage;
/* Private functions ---------------------------------------------------------*/
uint8_t  TXCAN_Protol[8] = {0};
uint8_t  TXCAN_Protol1[8] = {0};
uint8_t  TXCAN_Protol2[8] = {0};

uint8_t  RXCAN_Protol[8] = {0};
uint8_t  RXCAN_Protol1[8] = {0};  ////对接收运动板CAN变量进行分类
uint8_t  RXCAN_Protol11[8] = {0}; 
uint8_t  RXCAN_Protol12[8] = {0}; 
uint8_t  RXCAN_Protol2[8] = {0};
uint8_t  RXCAN_Protol3[8] = {0};
u8 can_send_buff[8] = {1, 2, 3, 4, 5, 6, 7, 0Xff};//最后一个数组变量0xff最后一个数据标示位

void CAN_GPIO_Configuration(void)  //CAN 通信管脚配置 // PA.11--CANRX   PA.12--CANTX
{



//GPIO config
    GPIO_InitTypeDef GPIO_InitStructure;

// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* Configure CAN pin: RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN1_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_Port_CAN1, &GPIO_InitStructure);

    /* Configure CAN pin: TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN1_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_Port_CAN1, &GPIO_InitStructure);

}
///**********************************
///name: void CAN_Configuration(void)
///funcion: CAN configure
///**********************************/

void CAN_Configuration(void)
{

    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    /* CAN register init */
    CAN_DeInit(CAN1);                                    //复位CAN1的所有寄存器
    /* Struct init*/
    CAN_StructInit(&CAN_InitStructure);                  //将寄存器全部设置成默认值

    /* CAN1  cell init */
    CAN_InitStructure.CAN_TTCM = DISABLE;                /*禁止时间触发通讯模式*/
    CAN_InitStructure.CAN_ABOM = DISABLE;                /*自动退出离线状态方式，0-相当于有条件手动离线，1-相当于自动离线*/
    CAN_InitStructure.CAN_AWUM = DISABLE;                /*0-由软件通过清0唤醒，1-检测到报文时，自动唤醒*/
    CAN_InitStructure.CAN_NART = ENABLE;                /*0-一直重复发送直到成功，1-不论成功以否只发送一次*/
    CAN_InitStructure.CAN_RFLM = DISABLE;                /*0-溢出时未锁定，新报文盖掉掉报文，1-FIFO锁定，溢出后新报文直接丢失*/
    CAN_InitStructure.CAN_TXFP = DISABLE;                 /*0-报文发送优先级由标志符决定，1-报文发送优先级由请求先后顺序决定*/
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;


    /* CAN Baudrate = 250KBps*/
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;              //设置重新同步跳转的时间量子
    CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;              //设置字段1的时间量子数
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;              //设置字段2的时间量子数
    CAN_InitStructure.CAN_Prescaler = 4;                  //配置时间量子长度为1周期    低波特率远距离传输更稳定    如果距离短可以调小这个值
    /*Initializes the CAN1  */ ///  36/(1+10+7)/4=500     
    CAN_Init(CAN1, &CAN_InitStructure);                   //用以上参数初始化CAN1的端口



    /* CAN1 filter init */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                                //选择CAN过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;              //初始化位标识/屏蔽模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;             //选择过滤器为32位
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;                           //过滤器标识号高16位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                            //过滤器标识号低16位
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;                       //根据模式选择过滤器标识号或屏蔽号的高16位
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;                        //根据模式选择过滤器标识号或屏蔽号的低16位
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;                        //将FIFO 0分配给过滤器0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;                       //使能过滤器
    CAN_FilterInit(&CAN_FilterInitStructure);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
//

}






/*******************************************************************************
* Function Name  : Init_RxMes
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void Init_RxMes(CanRxMsg *RxMessage)
{
    uint8_t i = 0;

    RxMessage->StdId = 0x00;
    RxMessage->ExtId = 0x00;
    RxMessage->IDE = 0;
    RxMessage->DLC = 0;
    RxMessage->FMI = 0;
    for (i = 0; i < 8; i++)
    {
        RxMessage->Data[i] = 0x00;
    }
}


void Init_TxMes(CanTxMsg *TxMessage)
{
    uint8_t i = 0;

    TxMessage->StdId = 0x00;
    TxMessage->ExtId = 0x00;
    TxMessage->IDE = 0;
    TxMessage->DLC = 0;

    for (i = 0; i < 8; i++)
    {
        TxMessage->Data[i] = 0;
    }
}
/*******************************************************************************
* Function Name  : CAN_TX
* Description    :
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void CAN_TX(uint8_t  *CAN_TX_Buf ,u32 can_id)
{
    uint8_t i = 0;
    unsigned char TransmitMailbox;
    /* transmit 1 message */
    TxMessage.StdId = can_id;
    TxMessage.ExtId = can_id;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.DLC = 8;
    for(i = 0; i < 8; i++)
    {

        TxMessage.Data[i] = CAN_TX_Buf[i];
    }

// CAN_Transmit(CAN1, &TxMessage);


    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    i = 0;
    while((CAN_TransmitStatus(CAN1, TransmitMailbox) == CANTXOK) && (i != 0xFF))
    {
        i++;
    }

    i = 0;
    while((CAN_MessagePending(CAN1, CAN_FIFO0) < 1) && (i != 0xFF))
    {
        i++;
    }
    Init_TxMes(&TxMessage);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
