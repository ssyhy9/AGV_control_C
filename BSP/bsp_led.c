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

#include "bsp_led.h"
#include "main.h"

/**
  * @brief          turn LED ON or OFF
  * @param[in]      led: the led that needs operating
  * @param[in]      state: ON or OFF
  * @retval         none
  */
void led_set(LED_List led, LED_State state)
{
    GPIO_TypeDef *gpiox;
    uint16_t gpio_pin;
    GPIO_PinState pin_state;

    if (state == LED_OFF)
    {
        pin_state = GPIO_PIN_SET;
    }
    else
    {
        pin_state = GPIO_PIN_RESET;
    }

    switch (led)
    {
        case LED_R:
            gpiox = LED_R_GPIO_Port;
            gpio_pin = LED_R_Pin;
            break;
        case LED_G:
            gpiox = LED_G_GPIO_Port;
            gpio_pin = LED_G_Pin;
            break;
        case LED_G1:
            gpiox = LED_G1_GPIO_Port;
            gpio_pin = LED_G1_Pin;
            break;
        case LED_G2:
            gpiox = LED_G2_GPIO_Port;
            gpio_pin = LED_G2_Pin;
            break;
        case LED_G3:
            gpiox = LED_G3_GPIO_Port;
            gpio_pin = LED_G3_Pin;
            break;
        case LED_G4:
            gpiox = LED_G4_GPIO_Port;
            gpio_pin = LED_G4_Pin;
            break;
        case LED_G5:
            gpiox = LED_G5_GPIO_Port;
            gpio_pin = LED_G5_Pin;
            break;
        case LED_G6:
            gpiox = LED_G6_GPIO_Port;
            gpio_pin = LED_G6_Pin;
            break;
        case LED_G7:
            gpiox = LED_G7_GPIO_Port;
            gpio_pin = LED_G7_Pin;
            break;
        case LED_G8:
            gpiox = LED_G8_GPIO_Port;
            gpio_pin = LED_G8_Pin;
            break;
        default:
            return;
    }
    HAL_GPIO_WritePin(gpiox, gpio_pin, pin_state);
}

