#include "ringbuf.h"

#include <stdlib.h>
#include "cmsis_os.h"

void RB_Init(RingBuffer *rb, size_t size)
{
  rb->external = false;
  rb->buffer = malloc(size);
  rb->head = 0;
  rb->tail = 0;
  rb->size = size;
}

void RB_Init_External(RingBuffer *rb, uint8_t *buf, size_t size)
{
  rb->external = true;
  rb->buffer = buf;
  rb->head = 0;
  rb->tail = 0;
  rb->size = size;
}

void RB_DeInit(RingBuffer *rb)
{
  rb->head = 0;
  rb->tail = 0;
  rb->size = 0;
  if (rb->external)
  {
    free(rb->buffer);
  }
  rb->buffer = NULL;
}

void RB_Reset(RingBuffer *rb)
{
  if (rb == NULL)
  {
    return;
  }
  rb->head = 0;
  rb->tail = 0;
}

bool RB_IsFull(RingBuffer *rb)
{
  if (rb == NULL)
  {
    return false;
  }
  return RB_Length(rb) == (rb->size - 1);
}

bool RB_IsEmpty(RingBuffer *rb)
{
  if (rb == NULL)
  {
    return false;
  }
  return (rb->head == rb->tail);
}

size_t RB_Size(RingBuffer *rb)
{
  if (rb == NULL)
  {
    return 0;
  }
  return rb->size - 1;
}

size_t RB_Length(RingBuffer *rb)
{
  if (rb == NULL)
  {
    return 0;
  }
  return (rb->size - rb->head + rb->tail) % rb->size;
}

int RB_Putc(RingBuffer *rb, uint8_t ch)
{
  if (rb == NULL)
  {
    return -1;
  }
  if (RB_IsFull(rb))
  {
    return -1;
  }

  rb->buffer[rb->tail] = ch;
  rb->tail = (rb->tail + 1) % rb->size;
  return ch;
}

int RB_Getc(RingBuffer *rb, uint32_t timeout)
{
  if (rb == NULL)
  {
    return -1;
  }
  uint32_t t = 0;
  for (; t < timeout; t++)
  {
    if (!RB_IsEmpty(rb))
    {
      break;
    }
    osDelay(1);
  }
  if (t < timeout)
  {
    char ch = rb->buffer[rb->head];
    rb->head = (rb->head + 1) % rb->size;
    return ch;
  }
  else
  {
    return -1;
  }
}
