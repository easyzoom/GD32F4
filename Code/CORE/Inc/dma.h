#ifndef __DMA_H_
#define __DMA_H_

#include "config.h"

#define USART0_RX_SIZE            150
#define ARRAYNUM(arr_name)        (uint32_t)(sizeof(arr_name)/sizeof(*(arr_name)))

void uart_dma_init(void);
void usart0_dma_send(uint8_t* buffer, uint16_t len);
void dma0_send(void);

#endif
