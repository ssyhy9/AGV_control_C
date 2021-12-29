/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid functions, including initialization and pid calculation functions
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Jan-25-2021     YW              1. modify to fit the project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef PID_H
#define PID_H

#include "arm_math.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;

    //three PID parameters
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;

    float32_t max_out;  //maximum output
    float32_t max_iout; //maximum integration output

    float32_t set;
    float32_t fdb;

    float32_t out;
    float32_t Pout;
    float32_t Iout;
    float32_t Dout;
    float32_t Dbuf[3];  //differentiation values |0:current |1:last time |2:time before last time
    float32_t error[3]; //error values 		 |0:current |1:last time |2:time before last time

} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data pointer
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: differential (delta) pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max output
  * @param[in]      max_iout: pid max integration output
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float32_t PID[3], float32_t max_out, float32_t max_iout);

/**
  * @brief          pid calculation
  * @param[out]     pid: PID struct data pointer
  * @param[in]      ref: feedback data 
  * @param[in]      set: setting value
  * @retval         pid output
  */
extern float32_t PID_calc(pid_type_def *pid, float32_t ref, float32_t set);

/**
  * @brief          clear pid output
  * @param[out]     pid: PID struct data pointer
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
