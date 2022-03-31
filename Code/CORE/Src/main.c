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

int main(void)
{
    systick_config();
    gpio_config();
    usart0_init(115200);
    usart3_init(115200);
    uart_dma_init();
    gpio_bit_write(GPIOA, GPIO_PIN_11, SET);
    while(1)
    {
        dma_send();
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
        delay_ms(500);
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART3, (uint32_t)ch);
    while(!usart_flag_get(UART3, USART_FLAG_TBE));
    return ch;
}

void DMA1_Stream5_USRHandler(void)
{
    if (dma_flag_get(DMA1, DMA_CH5, DMA_FLAG_FTF) != SET)
    {
        return;
    }
    dma_flag_clear(DMA1, DMA_CH5, DMA_FLAG_FTF);
//    usart0_dma_receive();
}

void DMA1_Stream7_USRHandler(void)
{
    if (dma_flag_get(DMA1, DMA_CH7, DMA_FLAG_FTF) != SET)
    {
        return;
    }
    dma_flag_clear(DMA1, DMA_CH7, DMA_FLAG_FTF);
    delay_ms(10);
}

void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
    {
        usart_data_receive(USART0);
        dma_channel_disable(DMA1, DMA_CH5);
    }
}

