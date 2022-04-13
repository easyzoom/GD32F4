#ifndef __DELAY_H_
#define __DELAY_H_

#include "gd32f4xx.h"

void time1_init(void);
void measure_runtime_start(void);
float measure_runtime_end(void);
void time1_delay_us(uint32_t us);
void time1_delay_ms(uint32_t ms);
#endif
