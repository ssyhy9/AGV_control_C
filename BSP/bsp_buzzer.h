/**
  ******************************************************************************
  * @file       bsp_buzzer.c/h
  * @brief      control buzzer. Turn on or turn off. The frequency and volume of
  *             sound can be set. To use this library, please configure TIM12
  *             channel 1 to PWM generation output mode before, use internal
  *             clock, prescalar is 1281
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Jan-27-2021     YW              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef BSP_BUZZER_H_
#define BSP_BUZZER_H_

#include "arm_math.h"

#define BUZZER_MAX_FREQ         2699.0f            //max frequency
#define BUZZER_MIN_FREQ         1.0f               //min frequency

/**
  * @brief          turn on buzzer
  * @param[in]      freq: buzzer sound frequency
  * @param[in]      vol: buzzer sound volume
  * @retval         none
  */
extern void buzzer_on(float32_t freq);

/**
  * @brief          turn off buzzer
  * @param          none
  * @retval         none
  */
extern void buzzer_off(void);

#endif /* BSP_BUZZER_H_ */
