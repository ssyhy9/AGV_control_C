#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <stm32f4xx_hal.h>
#include "cmsis_os.h"

#include "debugio.h"

extern osMutexId DBGSerialMutexHandle;
extern UART_HandleTypeDef huart8;

int dbgprintf(const char *format, ...)
{
  va_list _args;
  va_start(_args, format);
  va_list args;
  va_copy(args, _args);
  char buf[vsnprintf(NULL, 0, format, _args) + 1];
  va_end(_args);

  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (osMutexAcquire(DBGSerialMutexHandle, osWaitForever) == osOK)
  {
    HAL_UART_Transmit(&huart8, (uint8_t*) buf, strlen(buf), 0xFFFF);
    osMutexRelease(DBGSerialMutexHandle);
  }

  return strlen(buf);
}

HAL_StatusTypeDef dbgbuf(const uint8_t *buf, const size_t len)
{
  HAL_StatusTypeDef res = HAL_OK;
  if (osMutexAcquire(DBGSerialMutexHandle, osWaitForever) == osOK)
  {
    res = HAL_UART_Transmit(&huart8, (uint8_t*) buf, len, 0xFFFF);
    osMutexRelease(DBGSerialMutexHandle);
  }

  return res;
}
