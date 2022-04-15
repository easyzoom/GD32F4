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
#include "cmsis_os.h"
#include "lwip.h"
#include "switch_app.h"
#include "httpd.h"
#include "httpd_cgi_ssi.h"
#include "fs_api.h"
#include "delay.h"

void MX_FREERTOS_Init(void);

static uint32_t led_debug_1_tick = 0;
uint8_t reboot_flag = 0;
uint16_t reboot_timeout = 0;

void process_led_debug(void)
{
    if (xTaskGetTickCount() >= led_debug_1_tick + 500)
    {
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
        led_debug_1_tick = xTaskGetTickCount();
    }
}

void run_application_loop(void)
{
    gd32_lwip_init();
    switch_cfg();
    httpd_init();
    httpd_ssi_init();
    httpd_cgi_init();
    for (;;)
    {
        web_login_monitor();
        reboot();
        switch_app();
        process_led_debug();
        osDelay(1);
    }
}

int main(void)
{
    uint8_t state = 0;
    
    systick_config();
    time_delay_init();
    gpio_config();
    usart3_init(115200);
    gpio_bit_write(pinList[SW_RESET].port, pinList[SW_RESET].pin, SET);
    gd32_eth_gpio_init();
    enet_mac_dma_config();
    state = fs_api_init();
    sys_info_config(state);
    MX_FREERTOS_Init();
    osKernelStart();
    while(1)
    {
    }
}

 /* @brief 系统重启倒计时
 * 
 * @param [in] time 毫秒后系统重启，0无效
 * @return void
 */
void set_reboot(uint16_t time)
{
    reboot_timeout = time;
}

/**
 * @brief 系统重启处理
 * 
 * @param void
 * @return void
 */
void reboot(void)
{
    static uint16_t time = 0;
    if(reboot_timeout)
    {
        if(time++ > reboot_timeout)
        {
            reboot_flag = 1;
        }
    }
    if(reboot_flag)
    {
        taskENTER_CRITICAL();  /* 进入临界区 */
        NVIC_SystemReset(); // 复位
    }
}

/**
 * @brief 系统恢复默认设置
 * 
 * @param void
 * @return 
 */
int sys_default(void)
{
    int state = 0;
    
    state = fs_api_format();
    if(state)
    {
        LOG_PRINT_INFO("File system format error\r\n");    
    }
    set_reboot(200);
    return state;
}
