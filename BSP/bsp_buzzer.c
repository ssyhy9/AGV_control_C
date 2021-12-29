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

#include "bsp_buzzer.h"
#include "user_lib.h"
#include "main.h"

extern TIM_HandleTypeDef htim12;

/**
  * @brief          turn on buzzer
  * @param[in]      freq: buzzer sound frequency
  * @retval         none
  */
void buzzer_on(float32_t freq)
{
    //get APB1 timer clock
    float32_t apb1_tim_freq = (float32_t)HAL_RCC_GetPCLK1Freq() * 2.0f / (&htim12)->Instance->PSC;

    //constrain "freq"
    freq = fp32_constrain(freq, BUZZER_MIN_FREQ, BUZZER_MAX_FREQ);

    float32_t tim_arr = apb1_tim_freq / freq - 1.0f;
    float32_t tim_ccr = tim_arr * 0.5f;
    uint32_t tim_arr_given = (uint32_t)tim_arr;
    uint32_t tim_ccr_given = (uint32_t)tim_ccr;

    __HAL_TIM_SET_AUTORELOAD(&htim12, tim_arr_given);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, tim_ccr_given);

}

/**
  * @brief          turn off buzzer
  * @param          none
  * @retval         none
  */
void buzzer_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
}


