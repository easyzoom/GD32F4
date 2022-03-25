#ifndef __CAN_H_
#define __CAN_H_

#include "gd32f4xx.h"

void can_config_init(void);
void can_filter_config_init(void);
uint32_t can_get_txmailboxfreelevel(uint32_t can_periph);
#endif
