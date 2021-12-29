/*
 * protocol_def.h
 *
 *  Created on: Apr 5, 2021
 *      Author: Jason
 */

#ifndef VISION_PROTOCOL_DEF_H_
#define VISION_PROTOCOL_DEF_H_

//#define  CMD_DISABLE_AIMING       0x0000
//#define  CMD_ENABLE_AIMING        0x0001
//#define  CMD_GIMBAL_MOV_CHASSIS   0x0002
//#define  CMD_GIMBAL_MOV_GIMBAL    0x0003
//#define  CMD_GIMBAL_MOV_CURRENT   0x0004
//#define  CMD_SHOOT                0x0005
//#define  CMD_ENABLE_DET           0x0006
//#define  CMD_DISABLE_DET          0x0007
//#define  CMD_OFFER_DET_RLT        0x0008

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */


enum VisionCommandID
{
	DemoCmd = 0x0001
};

#pragma pack (push,1)

struct DemoCmd
{
	float dx;
	float dy;
};

union VisionCommandFrameData
{
	struct DemoCmd demoCmd;
};

struct VisionCommandFrame
{
	uint8_t SOF;
	uint16_t data_length;
	uint8_t sequence;
	uint8_t CRC8;
	uint16_t cmd_id;
	union VisionCommandFrameData data;
	uint16_t CRC16;
};
enum VisionCmdState
{
	SOF,
	DLEN,
	SEQ,
	CRC8,
	CMD,
	DATA,
	CRC16
};

#pragma pack (pop)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* VISION_PROTOCOL_DEF_H_ */
