/*!
    \file    main.c
    \brief   led spark with systick
    
    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "can.h"
#include "config.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "cm_backtrace.h"
#include "shell_port.h"

void MX_FREERTOS_Init(void);
extern void fault_test_by_unalign(void);
extern void fault_test_by_div0(void);
void log_init(void);

static uint32_t led_debug_1_tick = 0;

static void process_led_debug(void)
{
    if (xTaskGetTickCount() >= led_debug_1_tick + 500)
    {
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
        led_debug_1_tick = xTaskGetTickCount();
    }
}

void run_application_loop(void)
{
//    fault_test_by_unalign();
//    fault_test_by_div0();
    process_led_debug();
}

int main(void)
{
    gpio_config();
    usart0_init(115200);
    usart2_init(115200);
    usart3_init(115200);
    usart5_init(115200);
    uart_dma_init();
    can_config_init();
    systick_config();
    log_init();
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "Hello world !");
    cm_backtrace_init("CmBacktrace", HARDWARE_VERSION, SOFTWARE_VERSION);
    user_shell_init();
    /* Call init function for freertos objects (in freertos.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();
    while(1)
    {
    }
}

