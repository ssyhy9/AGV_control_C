/**
  ******************************************************************************
  * @file       bsp_laser.c/h
  * @brief      control laser on/off, and brightness.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Jan-29-2021     YW              1. done
  *
  @verbatim
  ==============================================================================
    To use this library, please set a pin as GPIO output and define
    its name as "LASER".
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "bsp_laser.h"
#include "main.h"

void laser_on(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

void laser_off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

