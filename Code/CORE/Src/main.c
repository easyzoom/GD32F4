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
//#define CAN0_USED
#define CAN1_USED

#ifdef  CAN0_USED
    #define CANX CAN0
    #define CAN_FIFOx CAN_FIFO0
#else 
    #define CANX CAN1
    #define CAN_FIFOx CAN_FIFO1
#endif
#if 1

void MX_FREERTOS_Init(void);

volatile ErrStatus test_flag;
volatile ErrStatus test_flag_interrupt;

enum
{
  PHASE_START                   = 0,
  PHASE_FLUSH_RX_MSG               ,
  PHASE_CHECK_TX_BOX               ,
  PHASE_SEND_REQ_MSG               ,
  PHASE_CHECK_RX_BOX               ,
  PHASE_RECV_RSP_MSG               ,
  PHASE_BUILDING_RET               ,
  PHASE_END                        ,
  PHASE_ERROR                      ,
};

static uint8_t                      ready[2] = {1, 1};
static can_trasnmit_message_struct  tx_head;
static can_receive_message_struct   rx_head;

static uint8_t __instruction_redirect_to_CAN(uint8_t *rx_buf, uint8_t rx_len, uint8_t *tx_buf, uint8_t *tx_len, uint32_t *phase_step, uint32_t *phase_tick)
{
  uint8_t wait_rsp          = (rx_buf[0] >> 7) & 0x1;
  uint8_t send_rsp          = (rx_buf[0] >> 6) & 0x1;
  uint8_t remote_frame      = (rx_buf[0] >> 4) & 0x1;
  uint8_t DLC               = (rx_buf[0]     ) & 0xf;;
  uint8_t controller_id     = rx_buf[1];
  uint8_t canopen_discard   = (rx_buf[2] >> 7) & 0x1;
  uint8_t wait_timeout      = (rx_buf[2] >> 3) & 0xf;
  uint16_t can_id           = (uint16_t)((((uint16_t)rx_buf[2] << 8) & 0x0700) |
                                         (((uint16_t)rx_buf[3] << 0) & 0x00ff));
    uint8_t transmit_mailbox = 4;
    uint32_t can_periph =
#ifdef ENABLE_INS_REDIRECT_TO_CAN_CAN1
                              controller_id == 0 ? CAN0 :
#endif
#ifdef ENABLE_INS_REDIRECT_TO_CAN_CAN2
                              controller_id == 1 ? CAN1 :
#endif
                              NULL;
                              
  uint32_t can_fifo_num = 
#ifdef ENABLE_INS_REDIRECT_TO_CAN_CAN1
                              controller_id == 0 ? CAN_FIFO0 :
#endif
#ifdef ENABLE_INS_REDIRECT_TO_CAN_CAN2
                              controller_id == 1 ? CAN_FIFO1 :
#endif
                              3;
                              
  if (can_periph == NULL || can_fifo_num == 3) //check if controller_id valid
  {
    printf("ERROR: invalid controller_id. %s:%d\r\n", __FUNCTION__, __LINE__);
    return 11; //invalid controller_id
  }

  if (DLC != (rx_len - 4) || DLC > 8) //check if DLC valid
  {
    printf("ERROR: invalid DLC. %s:%d\r\n", __FUNCTION__, __LINE__);
    return 12; //invalid Tx DLC
  }

  while (1)
  {
    switch (*phase_step)
    {
    case PHASE_START:
      {
        *phase_step = PHASE_FLUSH_RX_MSG;
      }
      break;
    case PHASE_FLUSH_RX_MSG:
      {
        /* flush rx message  */
        can_fifo_release(can_periph, can_fifo_num);
        *phase_step = PHASE_SEND_REQ_MSG;
        *phase_tick = xTaskGetTickCount();
      }
      break;
    case PHASE_SEND_REQ_MSG:
      {
        /* send instruction */
        {
          tx_head.tx_sfid   = can_id;
          tx_head.tx_efid   = 0x00U;
          tx_head.tx_ff   = (uint8_t)CAN_FF_STANDARD;
          tx_head.tx_ft   = (uint8_t)(remote_frame ? CAN_FT_REMOTE : CAN_FT_DATA);
          tx_head.tx_dlen   = DLC;
          for(int i = 0U; i < 8U; i++)
          {
              tx_head.tx_data[i] = rx_buf[4 + i];
          }
        }
        transmit_mailbox = can_message_transmit(can_periph, &tx_head);

        if (!wait_rsp) //check if need wait rsp
        {
          rx_head.rx_sfid = can_id;
          rx_head.rx_dlen = 0;
          *phase_step = PHASE_BUILDING_RET;
        }
        else
        {
          *phase_step = PHASE_CHECK_TX_BOX;
          *phase_tick = xTaskGetTickCount();
        }
      }
      break;
    case PHASE_CHECK_TX_BOX:
      {
        /* check tx mailbox  */
        if (CAN_TRANSMIT_OK == can_transmit_states(can_periph, transmit_mailbox))
        {
            *phase_step = PHASE_CHECK_RX_BOX;
            *phase_tick = xTaskGetTickCount();
        }
        else if (CAN_TRANSMIT_PENDING == can_transmit_states(can_periph, transmit_mailbox))
        {
            if (xTaskGetTickCount() > *phase_tick + 10)
            {
                printf("ERROR: TxMessage Timeout. %s:%d\r\n", __FUNCTION__, __LINE__);
                return 13; //TxMessage Timeout
            }
        }
        else
        {
            return 0xfb;
        }
      }
      break;
    case PHASE_CHECK_RX_BOX:
      {
        /* check rx mailbox  */
//        if (can_receive_message_length_get(can_periph, can_fifo_num) == 0)
//        {
          *phase_step = PHASE_RECV_RSP_MSG;
//        }
//        else
//        if (xTaskGetTickCount() > *phase_tick + (wait_timeout > 0 ? (wait_timeout*1) : 10))
//        {
//          printf("ERROR: RxMessage Timeout. %s:%d\r\n", __FUNCTION__, __LINE__);
//          return 15; //RxMessage Timeout
//        }
//        else
//        {
//          return 0xfb;
//        }
      }
      break;
    case PHASE_RECV_RSP_MSG:
      {
        /* receive response */
        can_message_receive(can_periph, can_fifo_num, &rx_head);
        if (canopen_discard && ((rx_head.rx_sfid & 0x0780) == 0x0080)) //emergency message
        {
          *phase_step = PHASE_CHECK_RX_BOX;
        }
        else
        if (!send_rsp) //check if need send rsp
        {
          rx_head.rx_dlen = 0;
          *phase_step = PHASE_BUILDING_RET;
        }
        else
        if (rx_head.rx_dlen > 8)
        {
          printf("ERROR: invalid DLC. %s:%d\r\n", __FUNCTION__, __LINE__);
          return 17; //invalid Rx DLC
        }
        else
        {
          *phase_step = PHASE_BUILDING_RET;
        }
      }
      break;
    case PHASE_BUILDING_RET:
      {
        if (*tx_len < 4 + rx_head.rx_dlen)
        {
            printf("ERROR: not enough space for result. %s:%d\r\n", __FUNCTION__, __LINE__);
            return 18; //not enough space for result
        }

        (*tx_len) = 0;

        tx_buf[(*tx_len)] = rx_head.rx_dlen;
        (*tx_len)++;

        tx_buf[(*tx_len)] = controller_id;
        (*tx_len)++;

        tx_buf[(*tx_len)] = (uint8_t)(((uint16_t)rx_head.rx_sfid >> 8) & 0xff);
        (*tx_len)++;

        tx_buf[(*tx_len)] = (uint8_t)(((uint16_t)rx_head.rx_sfid >> 0) & 0xff);
        (*tx_len)++;

        memcpy(&tx_buf[(*tx_len)], rx_head.rx_data, rx_head.rx_dlen);
        (*tx_len)+=rx_head.rx_dlen;
        printf("tx_buf[%d]: ", *tx_len);
        for(int size = 0; size <= *tx_len; size++)
        {
            printf("%02x ", tx_buf[size]);
        }
        printf("\r\n");

        *phase_step = PHASE_END;
      }
      break;
    case PHASE_END:
      return 0;
    default:
      return 0xfe; /* unknown phase!!! */
    }
  }
}

uint8_t instruction_redirect_to_CAN(uint8_t *rx_buf, uint8_t rx_len, uint8_t *tx_buf, uint8_t *tx_len, uint32_t *phase_step, uint32_t *phase_tick)
{
  if (*phase_step == 0 && ready[rx_buf[1]] != 1)
    return 0xfb;
  
  uint8_t ret = __instruction_redirect_to_CAN(rx_buf, rx_len, tx_buf, tx_len, phase_step, phase_tick);
  if (ret != 0xfb)
  {
    ready[rx_buf[1]] = 1;
  }
  else
  {
    ready[rx_buf[1]] = 0;   
  }
  
  return ret;
}

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
    transmit_message.tx_data[3] = 0x00;
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
//    systick_config();
    gpio_config();
    usart3_init(1500000);
    can_gpio_init();
//    nvic_config();
    can_config_init();
    can_filter_config_init();
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
//    test_flag = can_loopback();
//    printf("test_flag:%d\r\n", test_flag);
//    test_flag_interrupt = can_loopback_interrupt();
    MX_FREERTOS_Init();
    osKernelStart();
    while(1)
    {
    }
}

static uint32_t led_debug_1_tick = 0;
static void process_led_debug(void)
{
    if (xTaskGetTickCount() >= led_debug_1_tick + 500)
    {
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
        led_debug_1_tick = xTaskGetTickCount();
    }
}

void StartDefaultTask(void const * argument)
{
  
    for (;;)
    {
#if 1
        uint32_t test_step = 0;
        uint32_t *phase_step = &test_step;
#if 0
        //CAN1
        uint8_t rx_buf[12] = {0xc8, 0x00, 0xfe, 0x01, 0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
#if 1
        //CAN2
        uint8_t rx_buf[12] = {0xc8, 0x01, 0xfe, 0x01, 0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
        uint8_t tx_buf[20] = {0};
        uint8_t rx_len = 12, tx_len =12;
        uint8_t *p_tx_len = &tx_len;
        uint32_t test_tick = xTaskGetTickCount();
        uint32_t *phase_tick = &test_tick;
#endif
        process_led_debug();
#if 1
        instruction_redirect_to_CAN(rx_buf, rx_len, tx_buf, p_tx_len, phase_step, phase_tick);
        printf("tx_buf[%d]:", *p_tx_len);
        for(int i = 0; i <= *p_tx_len; i++)
        {
            printf("%02x ", tx_buf[i]);
        }
        printf("\r\n");
#endif
//        can_loopback();
        osDelay(1);
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART3, (uint32_t)ch);
    while(!usart_flag_get(UART3, USART_FLAG_TBE));
    return ch;
}
#else
/* select CAN baudrate */
/* 1MBps */
#define CAN_BAUDRATE  1000
/* 500kBps */
/* #define CAN_BAUDRATE  500 */
/* 250kBps */
/* #define CAN_BAUDRATE  250 */
/* 125kBps */
/* #define CAN_BAUDRATE  125 */
/* 100kBps */ 
/* #define CAN_BAUDRATE  100 */
/* 50kBps */ 
/* #define CAN_BAUDRATE  50 */
/* 20kBps */ 
/* #define CAN_BAUDRATE  20 */

FlagStatus can0_receive_flag;
FlagStatus can1_receive_flag;
FlagStatus can0_error_flag;
FlagStatus can1_error_flag;
can_parameter_struct can_init_parameter;
can_filter_parameter_struct can_filter_parameter;
can_trasnmit_message_struct transmit_message;
can_receive_message_struct receive_message;

void nvic_config(void);
void led_config(void);
void can_gpio_config(void);
void can_config(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    can0_receive_flag = RESET;
    can1_receive_flag = RESET;
    can0_error_flag = RESET;
    can1_error_flag = RESET;
    systick_config();
    gpio_config();
    /* configure GPIO */
    can_gpio_config();
    
    /* configure NVIC */
    nvic_config();
    
    /* configure USART */
    usart3_init(115200);
    
    printf("\r\nGD32F4xx dual CAN test, please press Wakeup key or Tamper key to start communication!\r\n");

    
    /* initialize CAN and filter */
    can_config(can_init_parameter, can_filter_parameter);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INT_RFNE0);
    can_interrupt_enable(CAN1, CAN_INT_RFNE0);
    
    /* initialize transmit message */
    transmit_message.tx_sfid = 0x300>>1;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 2;

    while(1){
        /* test whether the Tamper key is pressed */
            transmit_message.tx_data[0] = 0x55;
            transmit_message.tx_data[1] = 0xAA;
            printf("\r\n can0 transmit data:%x,%x", transmit_message.tx_data[0], transmit_message.tx_data[1]);
            /* transmit message */
            can_message_transmit(CAN0, &transmit_message);
            /* waiting for the Tamper key up */
        /* test whether the Wakeup key is pressed */
       
            transmit_message.tx_data[0] = 0xAA;
            transmit_message.tx_data[1] = 0x55;
            printf("\r\n can1 transmit data:%x,%x", transmit_message.tx_data[0], transmit_message.tx_data[1]);
            /* transmit message */
            can_message_transmit(CAN1, &transmit_message);
            /* waiting for the Wakeup key up */
        /* CAN0 receive data correctly, the received data is printed */
        if(SET == can0_receive_flag){
            can0_receive_flag = RESET;
            printf("\r\n can0 receive data:%x,%x", receive_message.rx_data[0], receive_message.rx_data[1]);
            
        }
        /* CAN1 receive data correctly, the received data is printed */
        if(SET == can1_receive_flag){
            can1_receive_flag = RESET;
            printf("\r\n can1 receive data:%x,%x", receive_message.rx_data[0], receive_message.rx_data[1]);
        }
        /* CAN0 error */
        if(SET == can0_error_flag){
            can0_error_flag = RESET;
            printf("\r\n can0 communication error");
        }
        /* CAN1 error */
        if(SET == can1_error_flag){
            can1_error_flag = RESET;
            printf("\r\n can1 communication error");
        }
    }
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
void can_config(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter)
{
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_INIT_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);
    can_deinit(CAN1);
    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    
    /* 1MBps */
#if CAN_BAUDRATE == 1000
    can_parameter.prescaler = 5;
    /* 500KBps */
#elif CAN_BAUDRATE == 500
    can_parameter.prescaler = 10;
    /* 250KBps */
#elif CAN_BAUDRATE == 250
    can_parameter.prescaler = 20;
    /* 125KBps */
#elif CAN_BAUDRATE == 125
    can_parameter.prescaler = 40;
    /* 100KBps */
#elif  CAN_BAUDRATE == 100
    can_parameter.prescaler = 50;
    /* 50KBps */
#elif  CAN_BAUDRATE == 50
    can_parameter.prescaler = 100;
    /* 20KBps */
#elif  CAN_BAUDRATE == 20
    can_parameter.prescaler = 250;
#else
    #error "please select list can baudrate in private defines in main.c "
#endif  
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    can_init(CAN1, &can_parameter);
    
    /* initialize filter */ 
    can_filter.filter_number=0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x3000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x3000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    
    can_filter_init(&can_filter);
    
    /* CAN1 filter number */
    can_filter.filter_number = 15;
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
    nvic_irq_enable(CAN0_RX0_IRQn,0,0);

    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX0_IRQn,1,1);
}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);
    
    /* configure CAN1 GPIO */
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_af_set(GPIOD, GPIO_AF_9, GPIO_PIN_0);
    
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_af_set(GPIOD, GPIO_AF_9, GPIO_PIN_1);
    
    /* configure CAN0 GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_5);
    
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_6);
}


int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART3, (uint32_t)ch);
    while(!usart_flag_get(UART3, USART_FLAG_TBE));
    return ch;
}
#endif
