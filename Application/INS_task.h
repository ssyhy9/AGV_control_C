/**
  ************************(C) COPYRIGHT 2021 UNNC LANCET************************
  * @file       INS_task.c/h
  * @brief      provide INS task and pointer request functions, based on
  *             bsp_imu.c/h library
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Feb-26-2021     YW              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ************************(C) COPYRIGHT 2021 UNNC LANCET************************
  */

#ifndef INS_TASK_H_
#define INS_TASK_H_

#include "main.h"
#include "arm_math.h"

#define BOARD_DOWN (1)                      //1->upper surface is front of board
                                            //0->upper surface is back of board

#define IST8310                             //comment this if there is no IST8310

#define INS_TASK_INIT_TIME 7                //delay for a while when at the beginning

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float32_t temp;

	float32_t wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float32_t wy;
	float32_t wz;

	float32_t vx;
	float32_t vy;
	float32_t vz;

	float32_t rol;
	float32_t pit;
	float32_t yaw;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu;

/**
  * @brief          imu task, init mpu6500, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void INS_task(void *pvParameters);

/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit: rad
  * @param[in]      none
  * @retval         the pointer of INS_angle
  */
extern const float32_t get_INS_angle_point(void);

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis, unit: rad/s
  * @param[in]      none
  * @retval         the pointer of INS_gyro
  */
extern const float32_t * get_gyro_data_point(void);

#endif


