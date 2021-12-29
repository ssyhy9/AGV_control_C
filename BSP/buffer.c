#include "buffer.h"

#include <stdlib.h>
#include <string.h>

void IO_Buffer_Init(IO_BufferTypeDef *buf, size_t size, osSemaphoreId sem)
{
  memset(buf, 0, sizeof(IO_BufferTypeDef));

  buf->external=false;
  buf->buffer = malloc(size);
  buf->size = size;
  buf->len = 0;
  buf->sem = sem;

  // clear semaphore
  if (buf->sem != NULL)
  {
    while (osSemaphoreAcquire(buf->sem, 0) == osOK)
    {
      ;
    }
  }
}

void IO_Buffer_Init_external(IO_BufferTypeDef *buf, uint8_t *mem, size_t size,
    osSemaphoreId sem)
{
  memset(buf, 0, sizeof(IO_BufferTypeDef));

  buf->external=true;
  buf->buffer = mem;
  buf->size = size;
  buf->len = 0;
  buf->sem = sem;

  // clear semaphore
  if (buf->sem != NULL)
  {
    while (osSemaphoreAcquire(buf->sem, 0) == osOK)
    {
      ;
    }
  }
}

void IO_Buffer_DeInit(IO_BufferTypeDef *buf)
{
  buf->external=false;
  buf->size = 0;
  buf->len = 0;
  buf->sem = NULL;

  if(buf->external)
  {
    free(buf->buffer);
  }
  buf->buffer=NULL;
}

osStatus IO_Buffer_Acquire(IO_BufferTypeDef *buf)
{
  if (buf == NULL)
  {
    return osErrorOS;
  }

  if (buf->sem != NULL)
  {
    return osSemaphoreAcquire(buf->sem, osWaitForever);
  }

  return osOK;
}

osStatus IO_Buffer_Release(IO_BufferTypeDef *buf)
{
  if (buf == NULL)
  {
    return osErrorOS;
  }

  if (buf->sem != NULL)
  {
    return osSemaphoreRelease(buf->sem);
  }

  return osOK;
}
