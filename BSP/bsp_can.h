#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "main.h"


typedef struct
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
} BSP_return;


/**
  * @brief		initialize CAN1 and CAN2 (to active receiving)
  * @param		none
  * @retval		none
  */
void BSP_CAN_Filtering_Init(void);
BSP_return *BSP_CAN_Msg_Ptr(void);

#endif /* _BSP_CAN_H_ */
