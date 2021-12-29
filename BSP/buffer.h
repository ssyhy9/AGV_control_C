#ifndef IO_BUFFER_H_
#define IO_BUFFER_H_

#include <stdbool.h>

#include "cmsis_os.h"

typedef struct __IO_BufferTypeDef
{
  bool external;
  uint8_t *buffer;
  size_t size;
  volatile size_t len;

  osSemaphoreId sem;
} IO_BufferTypeDef;

void IO_Buffer_Init(IO_BufferTypeDef *buf, size_t size, osSemaphoreId sem);
void IO_Buffer_Init_external(IO_BufferTypeDef *buf, uint8_t *mem, size_t size,
    osSemaphoreId sem);
void IO_Buffer_DeInit(IO_BufferTypeDef *buf);

osStatus IO_Buffer_Acquire(IO_BufferTypeDef *buf);
osStatus IO_Buffer_Release(IO_BufferTypeDef *buf);

#endif /* IO_BUFFER_H_ */
