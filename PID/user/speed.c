#include "speed.h"

uint8_t TxData[8] = {0};
int16_t speed = 500;
CAN_TxHeaderTypeDef CAN1_TxHander;

uint32_t* TxMailbox;  
void can1_play(void)
	{
		CAN1_TxHander .IDE = CAN_ID_STD ;
CAN1_TxHander .RTR = CAN_RTR_DATA ;
CAN1_TxHander .DLC = 0x08;
CAN1_TxHander .StdId = 0x200;

TxData[0] = speed >>8;
TxData[1] = speed ;
TxData[2] = speed >>8;
TxData[3] = speed;
TxData[4] = speed >>8;
TxData[5] = speed;
TxData[6] = speed >>8;
TxData[7] = speed;

HAL_CAN_AddTxMessage(&hcan1,&CAN1_TxHander,TxData,TxMailbox);



}
	
