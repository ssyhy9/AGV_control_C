/**
  ******************************************************************************
  * @file       bsp_laser.c/h
  * @brief      control laser on/off, and brightness.
  * @note       pin: J26-3
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

#ifndef BSP_LASER_H
#define BSP_LASER_H

extern void laser_on(void);
extern void laser_off(void);

#endif

