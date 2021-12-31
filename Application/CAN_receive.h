/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
//#include "bsp_can.h"

typedef enum {
	Motor_M3508_ID = 0x201,
	Motor_M2006_ID = 0x202,
	Current_Set_ID = 0x1FF,
} Control_Motor;

typedef enum {
	LeftUpper_Spd_ID = 0x401,
	LeftUpper_Ang_ID = 0x402,
	LeftLower_Spd_ID = 0x403,
	LeftLower_Ang_ID = 0x404,
	RightUpper_Spd_ID = 0x405,
	RightUpper_Ang_ID = 0x406,
	RightLower_Spd_ID = 0x407,
	RightLower_Ang_ID = 0x408,
	CenterBoard_CMD_ID = 0x3FF,
} CMD_ID;

typedef struct {
	CAN_RxHeaderTypeDef MotorRxMsgHeader;
	int16_t AngRaw;
	int16_t SpdRaw;
	int16_t TorqueCurrentRaw;
	int8_t TemperatureRaw;
} Motor_RxMsg_Raw;

typedef struct {
	CAN_TxHeaderTypeDef MotorTxMsgHeader;
	uint8_t DATA[8];
} Motor_TxMsg;

typedef struct {
	int16_t CurrentSet_Ang;
	int16_t CurrentSet_Spd;
} CenterBoard_CMD;

typedef struct {
	Control_Motor MotorID;
	Motor_RxMsg_Raw MotorRxMsgRaw[2][4];//2 motor informations for 4 board
	Motor_TxMsg MotorTxMsg;
	CenterBoard_CMD CenterBoardCMD;
	CMD_ID CMDID;
} Wheel_Board_Msg;

extern void APP_Get_CAN1_Data(CAN_RxHeaderTypeDef *Header, uint8_t DATA[]);
extern void APP_Get_CAN2_Data(CAN_RxHeaderTypeDef *Header, uint8_t DATA[]);

extern void BSP_Send_Msg_to_SideBoard(CAN_HandleTypeDef *hcan, uint16_t CommandID, CenterBoard_CMD Command[]);
extern void APP_Get_SideBoard_Data(uint8_t MotorIdx, uint16_t SideBoardIdx, uint8_t CAN_DATA[]) ;

#endif



