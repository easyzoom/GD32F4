#include "dma.h"

uint16_t ADC_ConvertedValue[9];


void uart_dma_init(void)
{
    dma_multi_data_parameter_struct dma_init_struct;
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA1);

    dma_deinit(DMA1,DMA_CH0);
    dma_init_struct.periph_addr = (uint32_t)(&ADC_SYNCDATA);
    dma_init_struct.periph_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr = (uint32_t)(&ADC_ConvertedValue);
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_burst_width = DMA_MEMORY_BURST_SINGLE;
    dma_init_struct.periph_burst_width = DMA_PERIPH_BURST_SINGLE;
    dma_init_struct.critical_value = DMA_FIFO_2_WORD;
    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.number = 9;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_multi_data_mode_init(DMA1,DMA_CH0,&dma_init_struct);
    dma_channel_subperipheral_select(DMA1,DMA_CH0,DMA_SUBPERI0);
    dma_circulation_disable(DMA1, DMA_CH0);
    dma_channel_enable(DMA1,DMA_CH0);
    
//    dma_deinit(DMA1,DMA_CH2);
//    dma_init_struct.periph_addr = (uint32_t)(&ADC_RDATA(ADC1)); 
//    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//    dma_init_struct.memory0_addr = (uint32_t)ADC_ConvertedValue2;
//    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
//    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
//    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
//    dma_init_struct.number = 3;
//    dma_init_struct.priority = DMA_PRIORITY_HIGH;
//    dma_single_data_mode_init(DMA1,DMA_CH2,&dma_init_struct);
//    dma_channel_subperipheral_select(DMA1,DMA_CH2,DMA_SUBPERI1);
//    dma_circulation_disable(DMA1, DMA_CH2);
//    dma_channel_enable(DMA1,DMA_CH2);
//    
//    dma_deinit(DMA1,DMA_CH1);
//    dma_init_struct.periph_addr = (uint32_t)(&ADC_RDATA(ADC2));
//    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//    dma_init_struct.memory0_addr = (uint32_t)ADC_ConvertedValue3;
//    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
//    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
//    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
//    dma_init_struct.number = 3;
//    dma_init_struct.priority = DMA_PRIORITY_HIGH;
//    dma_single_data_mode_init(DMA1,DMA_CH1,&dma_init_struct);
//    dma_channel_subperipheral_select(DMA1,DMA_CH1,DMA_SUBPERI2);
//    dma_circulation_disable(DMA1, DMA_CH1);
//    dma_channel_enable(DMA1,DMA_CH1);
}



