#ifndef __DELAY_H_
#define __DELAY_H_

#include "gd32f4xx.h"

void time_delay_init(void);
void time1_delay_ms(uint32_t ms);
void time1_delay_decrement(void);
#endif
