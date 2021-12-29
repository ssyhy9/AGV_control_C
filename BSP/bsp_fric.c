/**
  ******************************************************************************
  * @file       bsp_fric.c/h
  * @brief      control friction motor speed.
  * @note       attention, friction motors needs initialization, which should be
  *             done in shoot task library.
  *             pin: motor1->J26-1 motor2->J26-2
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Jan-28-2021     YW              1. done
  *
  @verbatim
  ==============================================================================
    To use this library, please set timer 4 channel 1 and channel 2 into
    PWM generation mode, set prescalar to 83, set counter period (ARR) to
    19999, add timer base start function and timer PWM start function
    (channel 1 and 2) in "tim.c".

    Attention, friction motors needs initialization, which should be done
    in shoot task library.
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "bsp_fric.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

/**
  * @brief          stop both two friction motors
  * @param          none
  * @retval         none
  */
void fric_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, FRIC_OFF);
}

/**
  * @brief          operate No.1 friction motor
  * @param[in]      cmd: it can be one of following values:
  *                         ->FRIC_UP       fast speed (also max speed)
  *                         ->FRIC_DOWN     slow speed
  *                         ->FRIC_OFF      stop
  * @retval         none
  */
void fric1_on(uint16_t cmd)
{
    TIM4->CCR1=cmd;
}

/**
  * @brief          operate No.2 friction motor
  * @param[in]      cmd: it can be one of following values:
  *                         ->FRIC_UP       fast speed (also max speed)
  *                         ->FRIC_DOWN     slow speed
  *                         ->FRIC_OFF      stop
  * @retval         none
  */
void fric2_on(uint16_t cmd)
{
    TIM4->CCR2=cmd;
}


