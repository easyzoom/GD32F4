#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>
#include "gd32f4xx.h"

#define DELAY_TIMER_RCU_PERIPH  RCU_TIMER1
#define DELAY_TIMER_PERIPH      TIMER1
#define DELAY_TIMER_NVIC_IRQ    TIMER1_IRQn
#define DELAY_TIMER_PRESCALER   1
#define DELAY_TIMER_PERIOD      41999

void  time_delay_init(void);
void time1_delay_ms(uint32_t ms);
void time1_delay_decrement(void);
#endif
