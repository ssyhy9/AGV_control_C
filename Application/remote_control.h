/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      to deal with remote controller, the remote controller uses
  *             SBUS-like protocol to achieve data transmission, here we use
  *             DMA to release CPU pressure, it use serial port idle interrupt
  *             function to handle data, and provide some functions like "restart
  *             offline DMA or serial" to make sure the system is stable when
  *             hot-plugging happens.
  * @note	    remember to add usart1 data transmission back soon.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *  V2.0.0     Jan-25-2021     YW              1. modify to fit the project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#define BSP_USART_LIB_NEED

#include "bsp_rc.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "CAN_receive.h"
#include "pid.h"


#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
#pragma pack (push,1)
typedef struct
{
	struct
	{
		/**
		  * int16_t ch[0]: channel 0
		  * int16_t ch[1]: channel 1
		  * int16_t ch[2]: channel 2
		  * int16_t ch[3]: channel 3
		  * int16_t ch[4]: channel 4
		  */
		int16_t ch[5];
		/**
		  * s[0]: switch right
		  * s[1]: switch left
		  */
		char s[2];
	}rc;
	struct
	{
		int16_t x;		//mouse X axis
		int16_t y;		//mouse Y axis
		int16_t z;		//mouse Z axis
		uint8_t press_l;	//mouse Left is pressed
		uint8_t press_r;	//mouse Right is pressed
	}mouse;
	struct
	{
		//each bit reps one key, see PC Key Definition section
		uint16_t v;		//PC keys
	}key;
	struct
	{
	    float32_t AI_X;
	    float32_t AI_Y;
	}Nuc_cmd;

}RC_ctrl_t;
#pragma pack (pop)

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);
#endif
