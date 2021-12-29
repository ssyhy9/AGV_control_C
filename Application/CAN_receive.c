/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       remember to add hook function back soon.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  V2.0.0     Jan-26-2021     YW              1. modify to fit this project
  *  V3.0.0     Oct-27-2021     SPY             1. AGV control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

//#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Wheel_Board_Msg CenterBoard;

// static void APP_Get_SideBoard_Data(uint8_t MotorIdx, uint16_t SideBoardID, uint8_t CAN_DATA[]);

/**
 * @brief:        CAN2 Data is passed from BSP Layer to APP Layer
 * @param[in]:    Header: the header get from BSP layer
 * @param[in]:    DATA[]: The data get from BSP layer
 * @retval:       None
 */

//void APP_Get_CAN1_Data(CAN_RxHeaderTypeDef *Header, uint8_t DATA[]) {
//	CenterBoard_CMD Command[];
//
//	switch (Header->StdId) {
//	case LeftUpper_Spd_ID:
//	case LeftUpper_Ang_ID: {
////		APP_Send_Msg_to_SideBoard(hcan1, LeftUpper_Spd_ID, Command[]);
////		APP_Send_Msg_to_SideBoard(hcan1, LeftUpper_Ang_ID, Command[]);
//		APP_Get_SideBoard_Data((Header->StdId - LeftUpper_Spd_ID), 0, DATA);
//		break;
//	}
//	case LeftLower_Spd_ID:
//	case LeftLower_Ang_ID: {
//		APP_Get_SideBoard_Data((Header->StdId - LeftLower_Spd_ID), 1, DATA);
//		break;
//	}
//	default:
//		break;
//	}
//}

//void APP_Get_CAN2_Data(CAN_RxHeaderTypeDef *Header, uint8_t DATA[]) {
//	switch (Header->StdId) {
//	case RightUpper_Spd_ID:
//	case RightUpper_Ang_ID:
//	{
//		APP_Get_SideBoard_Data((Header->StdId - RightUpper_Spd_ID), 2, DATA);
//		break;
//	}
//	case RightLower_Spd_ID:
//	case RightLower_Ang_ID: {
//		APP_Get_SideBoard_Data((Header->StdId - RightLower_Spd_ID), 3, DATA);
//		break;
//	}
//	default:
//		break;
//	}
//}


void APP_Get_SideBoard_Data(uint8_t MotorIdx, uint16_t SideBoardIdx, uint8_t CAN_DATA[]) {
	CenterBoard.MotorRxMsgRaw[MotorIdx][SideBoardIdx].AngRaw = (CAN_DATA[0] << 8) + CAN_DATA[1];
	CenterBoard.MotorRxMsgRaw[MotorIdx][SideBoardIdx].SpdRaw = (CAN_DATA[2] << 8) + CAN_DATA[3];
	CenterBoard.MotorRxMsgRaw[MotorIdx][SideBoardIdx].TorqueCurrentRaw = (CAN_DATA[4] << 8)+ CAN_DATA[5];
	CenterBoard.MotorRxMsgRaw[MotorIdx][SideBoardIdx].TemperatureRaw = (CAN_DATA[6] << 8)+ CAN_DATA[7];
}

void BSP_Send_Msg_to_SideBoard(CAN_HandleTypeDef *hcan, uint16_t *CommandID, CenterBoard_CMD Command[]) {
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t DATA[8] = { 0 };

	//Header Information
	TxHeader.StdId = *CommandID;//0x3FF
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	if(hcan == &hcan1){
	//The data send to upper board
	DATA[0] = Command[0].CurrentSet_Ang >> 8;
	DATA[1] = Command[0].CurrentSet_Ang;
	DATA[2] = Command[0].CurrentSet_Spd >> 8;
	DATA[3] = Command[0].CurrentSet_Spd;

	//The data send to lower board
	DATA[4] = Command[2].CurrentSet_Ang >> 8;
	DATA[5] = Command[2].CurrentSet_Ang;
	DATA[6] = Command[2].CurrentSet_Spd >> 8;
	DATA[7] = Command[2].CurrentSet_Spd;
	}
	else{
	//The data send to upper board
	DATA[0] = Command[1].CurrentSet_Ang >> 8;
	DATA[1] = Command[1].CurrentSet_Ang;
	DATA[2] = Command[1].CurrentSet_Spd >> 8;
	DATA[3] = Command[1].CurrentSet_Spd;

	//The data send to lower board
	DATA[4] = Command[3].CurrentSet_Ang >> 8;
	DATA[5] = Command[3].CurrentSet_Ang;
	DATA[6] = Command[3].CurrentSet_Spd >> 8;
	DATA[7] = Command[3].CurrentSet_Spd;
	}

	HAL_CAN_AddTxMessage(hcan, &TxHeader, DATA, &send_mail_box);
}

