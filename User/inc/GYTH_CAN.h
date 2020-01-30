#ifndef _GYTH_CAN_H_
#define _GYTH_CAN_H_



#define RCC_APB2Periph_GPIO_CAN1    RCC_APB2Periph_GPIOA
#define GPIO_Port_CAN1              GPIOA
#define GPIO_Pin_CAN1_RX            GPIO_Pin_11
#define GPIO_Pin_CAN1_TX            GPIO_Pin_12

extern            CanTxMsg TxMessage;
extern            CanRxMsg RxMessage;
extern unsigned char ret;
extern uint8_t  TXCAN_Protol[8];
extern uint8_t  TXCAN_Protol1[8];
extern uint8_t  TXCAN_Protol2[8];
extern u8 can_send_buff[8];
extern uint8_t  RXCAN_Protol[8];
extern uint8_t  RXCAN_Protol1[8];
extern uint8_t  RXCAN_Protol11[8];
extern uint8_t  RXCAN_Protol12[8];
extern uint8_t  RXCAN_Protol2[8];
extern uint8_t  RXCAN_Protol3[8];
void              CAN_GPIO_Configuration(void);
void              CAN_Configuration(void);
void              Init_RxMes(CanRxMsg *RxMessage);
void              Init_TxMes(CanTxMsg *TxMessage);

extern  unsigned char     CAN_RX(void);
extern  void CAN_TX(uint8_t  *CAN_TX_Buf ,u32 can_id);
#endif
