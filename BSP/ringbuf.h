#ifndef IO_RINGBUF_H_
#define IO_RINGBUF_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct ringbuf
{
  bool external;
  uint8_t *buffer;
  volatile size_t head;
  volatile size_t tail;
  volatile size_t size;
} RingBuffer;

void RB_Init(RingBuffer *rb, size_t size);
void RB_Init_External(RingBuffer *rb, uint8_t *buf, size_t size);
void RB_DeInit(RingBuffer *rb);
void RB_Reset(RingBuffer *rb);

bool RB_IsFull(RingBuffer *rb);
bool RB_IsEmpty(RingBuffer *rb);
size_t RB_Size(RingBuffer *rb);

size_t RB_Length(RingBuffer *rb);

int RB_Putc(RingBuffer *rb, uint8_t ch);
int RB_Getc(RingBuffer *rb, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* IO_RINGBUF_H_ */
