/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task
  * @note       1.remember to add detect_task.c/h
  *             2.remember to add INS_task.c/h
  *             3.remember to add chassis_power_control.c/h
  *             4.remember to add gimbal_behaviour.c/h
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
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
//#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//at the beginning of task ,wait a period
#define CHASSIS_TASK_INIT_TIME 357

//the remote controller channel index of controlling vertical speed
#define CHASSIS_X_CHANNEL 1

//the remote controller channel index of controlling horizontal speed
#define CHASSIS_Y_CHANNEL 0

//in some mode, we can use remote controller to control its rotation speed
#define CHASSIS_WZ_CHANNEL 4

//the remote controller switch index of choosing chassis behavior mode
#define CHASSIS_MODE_CHANNEL 0

//ratio that joy stick value (max 660) change to vertical speed (m/s)
#define CHASSIS_X_RC_SEN 0.008f
//ratio that joy stick value (max 660) change to horizontal speed (m/s)
#define CHASSIS_Y_RC_SEN 0.008f
//in "not following yaw angle" mode, the ratio that joy stick value changed to chassis rotation speed
#define CHASSIS_WZ_RC_SEN 0.005f
//in "following yaw angle" mode, the ratio that joy stick value added to chassis angle
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//chassis speed (acceleration) smooth parameter (used in filter)
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//joy stick value dead zone (not become zero at center)
#define CHASSIS_RC_DEADLINE 10

//the distance between motor and center of robot
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time interval 2ms    //
#define CHASSIS_CONTROL_TIME_MS 2           //<-These two perhaps can be combined?
//chassis task control time interval 0.002s //
#define CHASSIS_CONTROL_TIME 0.002f         //
//chassis task control frequence, no use now
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//m3508 max can-sent control currentֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//the key (from keyboard) let chassis swing (CTRL)
#define SWING_KEY 			KEY_PRESSED_OFFSET_CTRL
//chassis forward, back, left, right key (from keyboard)
#define CHASSIS_FRONT_KEY 	KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY 	KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY 	KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY 	KEY_PRESSED_OFFSET_D

//ratio of m3508 speed (rpm) changing to MA wheel speed (m/s),
#define M3508_MOTOR_RPM_TO_VECTOR   0.000415809748903494517209f

//single chassis motor max speed
#define MAX_WHEEL_SPEED   3.5f

//MA motor speed PID
#define M3508_MOTOR_SPEED_PID_KP 1200.0f     //15000
#define M3508_MOTOR_SPEED_PID_KI 6.0f        //10
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//(from gimbal yaw speed) MB motor speed close-loop PID parameters, max output and max integration output
#define M2006_MOTOR_SPEED_PID_KP        1000.0f		//500
#define M2006_MOTOR_SPEED_PID_KI        5.0f		//10
#define M2006_MOTOR_SPEED_PID_KD        0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT   500.0f        //5000.0f
#define M2006_MOTOR_SPEED_PID_MAX_IOUT  20.0f         //100.0f

//(from gimbal yaw) MB encoder angle close-loop PID parameters, max output and max integration output
#define M2006_ENCODE_RELATIVE_PID_KP        40.0f	//40
#define M2006_ENCODE_RELATIVE_PID_KI        1.0f
#define M2006_ENCODE_RELATIVE_PID_KD        0.0f	//7
#define M2006_ENCODE_RELATIVE_PID_MAX_OUT   100.0f	//200
#define M2006_ENCODE_RELATIVE_PID_MAX_IOUT  5.0f

//joy stick input dead zone, because the value is not 0 when it is in middle
#define RC_DEADBAND   10

//the ratio that changes motor angle value into ecd value
#ifndef RAD_TO_MOTOR_ECD
#define RAD_TO_MOTOR_ECD 1303.7972938f      //  8192 / (2*PI)
#endif

#ifndef HALF_PI
#define HALF_PI 1.57079632679490f		//PI/2
#endif


typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow the relative angle between gimbal and chassis
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle (chassis_yaw) close-looped control
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus directly.

} chassis_mode_e;

typedef struct
{
  float32_t speed;
  float32_t speed_set;

  uint16_t offset_ecd;            			 //motor middle place encoder value

  float32_t motor_ecd_speed;
  float32_t motor_ecd_speed_set;
  float32_t direction_wheel_angle;
  float32_t direction_wheel_angle_set;

  uint8_t wheel_dir_flag;
} chassis_board_t;


typedef struct
{
    const RC_ctrl_t *chassis_RC;				//the pointer to remote control

    chassis_mode_e chassis_mode;                //chassis control mode
    chassis_mode_e last_chassis_mode;           //last chassis control mode
    chassis_board_t chassis_board[4];           //chassis motor data

//    first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-pointֵ
//    first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point

    float32_t chassis_relative_angle;
    float32_t relative_angle_from_gimbal;
    float32_t vxy_relative_angle_to_gimbal;

	float32_t x_set;				  	   //get from "chassis_rc_to_control_vector"
    float32_t y_set;
    float32_t wz_set;
    float32_t x_channel_origin;				//the original value read from ch1
    float32_t y_channel_origin;				//the original value read from ch0
    float32_t wz_channel_origin;			//the original value read from ch4

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
extern void chassis_task(void *pvParameters);

/**
  * @brief          according to the channel value of remote controller, calculate
  *                 chassis vertical and horizontal speed set-point
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" pointer
  * @retval         none
  */
extern void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector);

#endif
