/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.0.0     Jan-26-2021     YW              1. modify to fit this project
  *  V3.0.0     Oct-27-2021     SPY             1. AGV control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

//disable chassis_power_control
#define CHASSIS_POWER_CONTROL_LIB_NEED

#include <math.h>
#include "chassis_task.h"
//#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "user_lib.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "INS_task.h"

#ifndef CHASSIS_POWER_CONTROL_LIB_NEED
#include "chassis_power_control.h"
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//deadband limit for channel 0, 1, 4 of remote controller that used in chassis control
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//chassis move data
chassis_move_t chassis_move;

//position of each wheel related to the chassis's coordinate in polar form (right-hand coordinate)
//polar angle of wheel 1~4 are listed with the same order: -PI/4, PI/4, 3PI/4, -3PI/4; the last term is the distance = 0.2m
const float32_t WHEEL_POSITION[5]  = {-0.78539816f, 0.78539816f, 2.35619448f, -2.35619448f, MOTOR_DISTANCE_TO_CENTER};

//prototype of all functions
static void chassis_init(chassis_move_t *chassis_move_init);

//static void chassis_set_mode(chassis_move_t *chassis_move_mode);

//static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

static void chassis_feedback_update(chassis_move_t *chassis_move_update);

void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector);

static void chassis_set_control(chassis_move_t *chassis_move_control);

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

static int32_t convert_angle_to_motor_ecd(float32_t input_angle);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;				//transplanted from DJI, though I don't know what it used for
#endif

CenterBoard_CMD CenterBoard_To_SideBoard_Data[4];
/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms), including:
  * 				->chassis initialization
  * 				->motor and rc data updating (in while(1))
  *					->set all the "set_point" according to AGV movement calculation
  *					->PID control
  *					->send all current command through CAN
  * @param[in]      pvParameters: null
  * @retval         none
  */
void chassis_task(void *pvParameters)
{
    //wait for a period initially
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    //chassis initialization
    chassis_init(&chassis_move);

    while (1)
    {
    	CAN_RxHeaderTypeDef rx_header;
//        //set chassis control mode
//        chassis_set_mode(&chassis_move);
//
//        //when mode changes, some data may needs changing
//        chassis_mode_change_control_transit(&chassis_move);

        //chassis data update
        chassis_feedback_update(&chassis_move);

        //set chassis control set-point
        chassis_set_control(&chassis_move);

        BSP_Send_Msg_to_SideBoard(&hcan1, &CenterBoard_CMD_ID, CenterBoard_To_SideBoard_Data);
        BSP_Send_Msg_to_SideBoard(&hcan2, &CenterBoard_CMD_ID, CenterBoard_To_SideBoard_Data);  //send message

        //OS delay
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif

//            }//end of "else"
//
//        }//end of detect-pending "if"

        //OS delay
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

    }//end of "while(1)"
}

/**
  * @brief          "chassis_move" initialization, including:
  *                 ->pid initialization
  *                 ->remote controller data pointer initialization
  *                 ->chassis move mode initialization
  *                 ->3508 chassis motors data pointer initialization
  *                 ->2006 chassis motor data pointer initialization
  *                 ->gyroscope sensor angle pointer initialization
  *                 ->maximum speed for chassis and each motor initialization
  * @param[out]     chassis_move_init: "chassis_move" pointer
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //get remote control data pointer
    chassis_move_init->chassis_RC = get_remote_control_point();
    
    // initialize of chassis mode (for test only, other modes would be added later)
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    
    //update data
    chassis_feedback_update(chassis_move_init);

}


/**
  * @brief          update some chassis measured data
  *                 such as rc data, euler angle, self-rotation speed, motor speed and robot speed
  * @param[out]     chassis_move_update: "chassis_move" pointer
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(chassis_move_update);

    //update the values return from remote controller
    chassis_move_update->chassis_RC = get_remote_control_point();

    //update the chassis relative angle
    chassis_move_update->chassis_relative_angle = get_INS_angle_point();

    //update the gimbal relative angle (give the value of 0, for test only)
    chassis_move_update->relative_angle_from_gimbal = 0;

}

/**
  * @brief          according to the channel value of remote controller, calculate
  *                 chassis vertical and horizontal speed set-point
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" pointer
  * @retval         none
  */
void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    int16_t x_channel;			//original data return from remote controller's channel 1
    int16_t y_channel;			//original data return from remote controller's channel 0
    int16_t wz_channel;			//original data return from remote controller's channel 4

    //update the values get from remote controller
    //pay attention to the sign of "y" here
    x_channel = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
    y_channel = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
    wz_channel = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];

    chassis_move_rc_to_vector->x_set = x_channel * CHASSIS_X_RC_SEN;
    chassis_move_rc_to_vector->y_set = y_channel * CHASSIS_Y_RC_SEN;
    chassis_move_rc_to_vector->wz_set = wz_channel * CHASSIS_WZ_RC_SEN;

    chassis_move_rc_to_vector->x_channel_origin = x_channel;
    chassis_move_rc_to_vector->y_channel_origin = y_channel;
    chassis_move_rc_to_vector->wz_channel_origin = wz_channel;

    //dead zone limit, because some remote control need be calibrated
    //the value of joy stick is not zero in middle place
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], x_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], y_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

}

/**
 * @brief	chassis_move_mode selection
 */
static void chassis_set_control(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_feedback_update(chassis_move_control);
    chassis_rc_to_control_vector(chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
    	float32_t vxy_to_gimbal, gimbal_angle_to_chassis;
    	float32_t vx_to_wheel_system, vy_to_wheel_system;
    	float32_t relative_angle_set_to_get;

    	vxy_to_gimbal = sqrt(pow(chassis_move_control->x_set, 2) + pow(chassis_move_control->y_set, 2));
    	gimbal_angle_to_chassis = chassis_move_control->relative_angle_from_gimbal - chassis_move_control->chassis_relative_angle;

    	float32_t vxy_to_gimbal_f32[4] =
    	{
    	  chassis_move_control->x_set, chassis_move_control->y_set, 0.0, 1.0
    	};

    	for (uint8_t i = 0; i < 4; i++)
    	{
    		vx_to_wheel_system = vxy_to_gimbal * cos(chassis_move_control->vxy_relative_angle_to_gimbal - (gimbal_angle_to_chassis)) + chassis_move_control->wz_set * sin(WHEEL_POSITION[i]);
    		vy_to_wheel_system = vxy_to_gimbal * sin(chassis_move_control->vxy_relative_angle_to_gimbal - (gimbal_angle_to_chassis)) + chassis_move_control->wz_set * cos(WHEEL_POSITION[i]);

//    		chassis_move_control->motor_chassis[i].wheel_dir_flag = 0;
//    		chassis_move_control->motor_chassis[i].speed_set = sqrt(pow(vx_to_chassis, 2) + pow(vy_to_chassis, 2));
    		//for single wheel system test only
    		chassis_move_control->chassis_board[i].speed_set = vx_to_wheel_system;
    		chassis_move_control->chassis_board[i].motor_ecd_speed_set = vy_to_wheel_system;

    		if (vy_to_wheel_system < 0) {
    			chassis_move_control->chassis_board[i].direction_wheel_angle_set = - acos(vx_to_wheel_system / chassis_move_control->chassis_board[i].speed_set);
    		}
    		else {
    			chassis_move_control->chassis_board[i].direction_wheel_angle_set = acos(vx_to_wheel_system / chassis_move_control->chassis_board[i].speed_set);
    		}

    		relative_angle_set_to_get = fabs(chassis_move_control->chassis_board[i].direction_wheel_angle_set) - fabs(chassis_move_control->chassis_board[i].direction_wheel_angle);

    		if (fabs(chassis_move_control->chassis_board[i].speed) > 2.5f && fabs(relative_angle_set_to_get) > HALF_PI)
    		{
    			continue;
    		}

    		if (fabs(chassis_move_control->chassis_board[i].speed) <= 2.5f && fabs(relative_angle_set_to_get) > HALF_PI && chassis_move_control->chassis_board[i].wheel_dir_flag == 0)
    		{
    			chassis_move_control->chassis_board[i].speed_set = 0;
    			chassis_move_control->chassis_board[i].direction_wheel_angle_set = - chassis_move_control->chassis_board[i].direction_wheel_angle_set;

    			chassis_feedback_update(chassis_move_control);
    			if (fabs(relative_angle_set_to_get) < 0.05f) chassis_move_control->chassis_board[i].speed_set = - chassis_move_control->chassis_board[i].speed_set;
    		}

    		chassis_move_control->chassis_board[i].wheel_dir_flag = 1;

//    		typedef struct {
//    			int16_t CurrentSet_Ang;
//    			int16_t CurrentSet_Spd;
//    		} CenterBoard_CMD;
//    		CenterBoard_CMD CenterBoard_To_SideBoard_Data[4];
    		CenterBoard_To_SideBoard_Data[i].CurrentSet_Spd = chassis_move_control->chassis_board[i].speed_set;
    		CenterBoard_To_SideBoard_Data[i].CurrentSet_Ang = chassis_move_control->chassis_board[i].direction_wheel_angle_set;
    	}

    }//end of "CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW"
}

/**
  * @brief          convert relative angle to ecd value
  * @param[in]      motor_chassis[j].wheel_relative_angle_set
  * @retval         relative angle (ecd_value)
  */
static int32_t convert_angle_to_motor_ecd(float32_t input_angle)
{
    return input_angle * RAD_TO_MOTOR_ECD;
}
