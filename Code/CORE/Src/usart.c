#include "usart.h"

void usart0_init(uint32_t baudrate)
{
    rcu_periph_clock_enable(RCU_USART0);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, baudrate);
    usart_parity_config(USART0,USART_PM_NONE);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0,USART_STB_1BIT);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    nvic_irq_enable(USART0_IRQn, 2, 0);
    usart_interrupt_enable(USART0, USART_INT_IDLE);
    usart_enable(USART0);
}

void usart2_init(uint32_t baudrate)
{
    rcu_periph_clock_enable(RCU_USART2);
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_8);
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_9);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    usart_deinit(USART2);
    usart_baudrate_set(USART2, baudrate);
    usart_parity_config(USART2,USART_PM_NONE);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2,USART_STB_1BIT);
    usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    nvic_irq_enable(USART2_IRQn, 2, 0);
    usart_interrupt_enable(USART2, USART_INT_IDLE);
    usart_enable(USART2);
}

void usart3_init(uint32_t baudrate)
{
    rcu_periph_clock_enable(RCU_UART3);
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_10);
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_11);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    usart_deinit(UART3);
    usart_baudrate_set(UART3, baudrate);
    usart_parity_config(UART3,USART_PM_NONE);
    usart_word_length_set(UART3, USART_WL_8BIT);
    usart_stop_bit_set(UART3,USART_STB_1BIT);
    usart_hardware_flow_rts_config(UART3, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(UART3, USART_CTS_DISABLE);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    nvic_irq_enable(UART3_IRQn, 2, 0);
    usart_interrupt_enable(UART3, USART_INT_IDLE);
    usart_enable(UART3);
}

void usart5_init(uint32_t baudrate)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART5);
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_6);
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_7);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    usart_deinit(USART5);
    usart_baudrate_set(USART5, baudrate);
    usart_parity_config(USART5,USART_PM_NONE);
    usart_word_length_set(USART5, USART_WL_8BIT);
    usart_stop_bit_set(USART5,USART_STB_1BIT);
    usart_hardware_flow_rts_config(USART5, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART5, USART_CTS_DISABLE);
    usart_receive_config(USART5, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART5, USART_TRANSMIT_ENABLE);
    nvic_irq_enable(USART5_IRQn, 2, 0);
    usart_interrupt_enable(USART5, USART_INT_IDLE);
    usart_enable(USART5);
}

void serial_transmit(uint32_t usart_periph, uint8_t *data,uint8_t length)
{
   uint8_t i =0;
   for(i = 0; i<length ;i++ )
  {
     usart_data_transmit(usart_periph, data[i]);
     while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
  }
}

void serial_receive(uint32_t usart_periph, uint8_t *data,uint8_t length)
{
   uint8_t i =0;
   for(i = 0; i<length ;i++ )
  {
     usart_data_transmit(usart_periph, data[i]);
     while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
  }
}


void UsartPrintf(uint32_t usart_periph, char *fmt,...)
{
    unsigned char UsartPrintfBuf[296];
    va_list ap;
    unsigned char *pStr = UsartPrintfBuf;

    va_start(ap, fmt);
    vsprintf((char *)UsartPrintfBuf, fmt, ap);
    va_end(ap);

    while(*pStr != 0)
    {
        usart_data_transmit(usart_periph, *pStr++);
        while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
    }

}


