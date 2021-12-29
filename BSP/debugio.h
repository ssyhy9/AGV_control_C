#ifndef IO_DEBUGIO_H_
#define IO_DEBUGIO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stm32f4xx_hal.h>

int dbgprintf(const char *format, ...);
HAL_StatusTypeDef dbgbuf(const uint8_t *buf, const size_t len);

#ifdef __cplusplus
}
#endif

#endif /* IO_DEBUGIO_H_ */
