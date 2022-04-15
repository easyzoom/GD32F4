#ifndef __MAIN_H
#define __MAIN_H

#include "config.h"

void reboot(void);
void set_reboot(uint16_t time);
int sys_default(void);

void run_application_loop(void);
#endif /* __MAIN_H */


