#ifndef __USART_H_
#define __USART_H_

#include "gd32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void usart0_init(uint32_t baudrate);
void usart2_init(uint32_t baudrate);
void usart3_init(uint32_t baudrate);
void usart5_init(uint32_t baudrate);

void serial_transmit(uint32_t usart_periph, uint8_t *data,uint8_t length);
void serial_receive(uint32_t usart_periph, uint8_t *data,uint8_t length);
void UsartPrintf(uint32_t usart_periph, char *fmt,...);

#endif
