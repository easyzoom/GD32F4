#include "dma.h"

uint8_t rx_buffer[USART0_RX_SIZE];
uint8_t tx_buffer[]="\n\rUSART DMA receive and transmit example, please input 10 bytes:\r\n";

void uart_dma_init(void)
{
    dma_single_data_parameter_struct dma_init_struct;
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA1);
    /* deinitialize DMA channel5 */
    dma_deinit(DMA1, DMA_CH5);
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.memory0_addr  = NULL;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = NULL;
    dma_init_struct.periph_addr = USART0_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(DMA1, DMA_CH5, &dma_init_struct);
    dma_channel_subperipheral_select(DMA1, DMA_CH5, DMA_SUBPERI4);
    dma_circulation_disable(DMA1, DMA_CH5);
    
    /* deinitialize DMA channel7 */
    dma_deinit(DMA1, DMA_CH7);
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.memory0_addr = NULL;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width  = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = NULL;
    dma_init_struct.periph_addr = USART0_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(DMA1, DMA_CH7, &dma_init_struct);
    dma_channel_subperipheral_select(DMA1, DMA_CH7, DMA_SUBPERI4);
    dma_circulation_disable(DMA1, DMA_CH7);

    usart_dma_receive_config(USART0, USART_DENR_ENABLE);
    nvic_irq_enable(DMA1_Channel5_IRQn, 1, 0);
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    nvic_irq_enable(DMA1_Channel7_IRQn, 2, 0);
    dma_interrupt_enable(DMA1, DMA_CH7, DMA_CHXCTL_FTFIE);
    dma_interrupt_enable(DMA1, DMA_CH5, DMA_CHXCTL_FTFIE);
    dma_periph_address_config(DMA1, DMA_CH1, USART0_DATA_ADDRESS);
    dma_channel_enable(DMA1, DMA_CH5);
    dma_channel_enable(DMA1, DMA_CH7);
}

void dma_send(void)
{
    usart0_dma_send(tx_buffer, strlen((char *)tx_buffer));
}

void usart0_dma_send(uint8_t* buffer, uint16_t len)
{
    dma_channel_disable(DMA1, DMA_CH1);
    dma_memory_address_config(DMA1, DMA_CH1, 0, (uint32_t)buffer);
    dma_transfer_number_config(DMA1, DMA_CH1, len);
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    dma_channel_enable(DMA1, DMA_CH1);
    while(RESET == dma_flag_get(DMA1, DMA_CH1, DMA_FLAG_FTF));
}

void usart0_dma_receive(void)
{
    printf("MSG_TRANS_BUF %d DATA RX:", dma_transfer_number_get(DMA1, DMA_CH5));
    for (uint8_t i = 0; i < USART0_RX_SIZE - dma_transfer_number_get(DMA1, DMA_CH5); i++)
    {
        printf( " %02X", rx_buffer[i]);
    }
    printf("\r\n");
}


