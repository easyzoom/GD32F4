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

#define CAN0_USED
//#define CAN1_USED

#ifdef  CAN0_USED
    #define CANX CAN0
    #define CAN_FIFOx CAN_FIFO0
#else 
    #define CANX CAN1
    #define CAN_FIFOx CAN_FIFO1
#endif
#if 1
FlagStatus can0_receive_flag;
FlagStatus can1_receive_flag;
FlagStatus can0_error_flag;
FlagStatus can1_error_flag;
can_trasnmit_message_struct transmit_message;
can_receive_message_struct receive_message;
volatile ErrStatus test_flag;
volatile ErrStatus test_flag_interrupt;

ErrStatus can_loopback(void)
{
    ErrStatus state = ERROR;
    can_trasnmit_message_struct transmit_message;
    can_receive_message_struct  receive_message;
    uint32_t timeout = 0xFFFF;
    uint8_t transmit_mailbox = 0;

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = 0x601;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    transmit_message.tx_data[0] = 0x40;
    transmit_message.tx_data[1] = 0x41;
    transmit_message.tx_data[2] = 0x60;
    transmit_message.tx_data[3] = 0x11;
    transmit_message.tx_data[4] = 0x00;
    transmit_message.tx_data[5] = 0x00;
    transmit_message.tx_data[6] = 0x00;
    transmit_message.tx_data[7] = 0x00;
    
    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);
    
    /* transmit message */
    transmit_mailbox = can_message_transmit(CANX, &transmit_message);
    /* waiting for transmit completed */
    while((CAN_TRANSMIT_OK != can_transmit_states(CANX, transmit_mailbox)) && (0 != timeout)){
        timeout--;
        if(timeout == 0)
        {
            printf("can time out\r\n");
        }
    }
    timeout = 0xFFFF;
    /* waiting for receive completed */
    while((can_receive_message_length_get(CANX, CAN_FIFOx) < 1) && (0 != timeout)){
        timeout--; 
    }

    /* initialize receive message*/
    receive_message.rx_sfid = 0x00;
    receive_message.rx_ff = 0;
    receive_message.rx_dlen = 8;
    for(int i = 0; i <= 7; i++)
    {
        receive_message.rx_data[i] = 0x00;
    }
    can_message_receive(CANX, CAN_FIFOx, &receive_message);
    
    /* check the receive message */
//    if((0x11 == receive_message.rx_sfid) && (CAN_FF_STANDARD == receive_message.rx_ff)
//       && (2 == receive_message.rx_dlen) && (0xCDAB == (receive_message.rx_data[1]<<8|receive_message.rx_data[0]))){
//        return SUCCESS;
//    }else{
//        return ERROR;
//    }
    printf("rx_dlen:%d\r\n",receive_message.rx_dlen);
    for(int i = 0; i <= 7; i++)
    {
        printf("%02x ",receive_message.rx_data[i]);
        state = SUCCESS;
    }
    printf("\r\n");
    return state;
}

void nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX1_IRQn,0,0);
    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX1_IRQn,1,0);
}

int main(void)
{
    systick_config();
    gpio_config();
    usart3_init(115200);
    can_gpio_init();
    can_config_init();
    nvic_config();
    
    can_filter_config_init();
    test_flag = can_loopback();
//    printf("test_flag:%d\r\n", test_flag);
//    test_flag_interrupt = can_loopback_interrupt();
    while(1)
    {
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
#else
volatile ErrStatus test_flag;
volatile ErrStatus test_flag_interrupt;

void nvic_config(void);
void led_config(void);
ErrStatus can_loopback(void);
ErrStatus can_loopback_interrupt(void);
void can_loopback_init(void);
/*!
    \brief      function for CAN loopback communication
    \param[in]  none
    \param[out] none
    \retval     ErrStatus
*/
ErrStatus can_loopback(void)
{
    ErrStatus state = ERROR;
    can_trasnmit_message_struct transmit_message;
    can_receive_message_struct  receive_message;
    uint32_t timeout = 0xFFFF;
    uint8_t transmit_mailbox = 0;
    /* initialize CAN */
    can_gpio_init();
    can_loopback_init();

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
    transmit_message.tx_sfid = 0x601;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    transmit_message.tx_data[0] = 0x40;
    transmit_message.tx_data[1] = 0x41;
    transmit_message.tx_data[2] = 0x60;
    transmit_message.tx_data[3] = 0x66;
    transmit_message.tx_data[4] = 0x00;
    transmit_message.tx_data[5] = 0x00;
    transmit_message.tx_data[6] = 0x00;
    transmit_message.tx_data[7] = 0x00;
    
    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &receive_message);
    
    /* transmit message */
    transmit_mailbox = can_message_transmit(CANX, &transmit_message);
    
    /* waiting for transmit completed */
    while((CAN_TRANSMIT_OK != can_transmit_states(CANX, transmit_mailbox)) && (0 != timeout)){
        timeout--;
        if(timeout == 0)
        {
            printf("can time out\r\n");
        }
    }
    timeout = 0xFFFF;
    /* waiting for receive completed */
    while((can_receive_message_length_get(CANX, CAN_FIFOx) < 1) && (0 != timeout)){
        timeout--; 
    }

    /* initialize receive message*/
    receive_message.rx_sfid = 0x00;
    receive_message.rx_ff = 0;
    receive_message.rx_dlen = 8;
    for(int i = 0; i <= 7; i++)
    {
        receive_message.rx_data[i] = 0x00;
    }
    can_message_receive(CANX, CAN_FIFOx, &receive_message);
    
    /* check the receive message */
//    if((0x11 == receive_message.rx_sfid) && (CAN_FF_STANDARD == receive_message.rx_ff)
//       && (2 == receive_message.rx_dlen) && (0xCDAB == (receive_message.rx_data[1]<<8|receive_message.rx_data[0]))){
//        return SUCCESS;
//    }else{
//        return ERROR;
//    }
    printf("rx_dlen:%d\r\n",receive_message.rx_dlen);
    for(int i = 0; i <= 7; i++)
    {
        printf("%02x ",receive_message.rx_data[i]);
        state = SUCCESS;
    }
    return state;
}

/*!
    \brief      initialize CAN and filter
    \param[in]  can_parameter
      \arg        can_parameter_struct
    \param[in]  can_filter
      \arg        can_filter_parameter_struct
    \param[out] none
    \retval     none
*/
void can_loopback_init(void)
{
    can_parameter_struct        can_parameter;
    can_filter_parameter_struct can_filter;
    
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    
    /* initialize CAN register */
    can_deinit(CANX);
    
    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.no_auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_LOOPBACK_MODE;
    /* configure baudrate to 500kbps */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.prescaler = 5;
    can_init(CANX, &can_parameter);

    /* initialize filter */
#ifdef  CAN0_USED
    /* CAN0 filter number */
    can_filter.filter_number = 0;
#else
    /* CAN1 filter number */
    can_filter.filter_number = 15;
#endif
    /* initialize filter */    
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFOx;
    can_filter.filter_enable=ENABLE;
    can_filter_init(&can_filter);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX1_IRQn,0,0);
    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX1_IRQn,0,0);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();
    gpio_config();
    can_config_init();
    can_filter_config_init();
    usart3_init(115200);
    /* enable CAN clock */
   
    /* configure NVIC */
    nvic_config();
    test_flag = can_loopback();
    printf("test_flag:%d\r\n", test_flag);
    /* loopback of interrupt */
//    test_flag_interrupt = can_loopback_interrupt();
//    printf("test_flag:%d\r\n", test_flag_interrupt);
    while (1)
    {
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
#endif
