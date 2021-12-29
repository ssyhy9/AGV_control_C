#ifndef _BSP_RC_H_
#define _BSP_RC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

#endif
