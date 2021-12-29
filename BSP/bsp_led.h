/**
  ******************************************************************************
  * @file       bsp_led.c/h
  * @brief      to turn LED ON or OFF. To use this library, users need to
  *             configure LED's GPIO in CubeMX before.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Jan-27-2021     YW              1. done
  *
  ******************************************************************************
  */

#ifndef BSP_LED_H_
#define BSP_LED_H_

typedef enum
{
    LED_R   = 0x09U,
    LED_G   = 0x00U,
    LED_G1  = 0x01U,
    LED_G2  = 0x02U,
    LED_G3  = 0x03U,
    LED_G4  = 0x04U,
    LED_G5  = 0x05U,
    LED_G6  = 0x06U,
    LED_G7  = 0x07U,
    LED_G8  = 0x08U
} LED_List;

typedef enum
{
    LED_OFF = 0x00U,
    LED_ON
} LED_State;

/**
  * @brief          turn LED ON or OFF
  * @param[in]      led: the led that needs operating
  * @param[in]      state: ON or OFF
  * @retval         none
  */
void led_set(LED_List led, LED_State state);

#endif /* BSP_LED_H_ */
