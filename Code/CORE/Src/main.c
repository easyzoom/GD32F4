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
#include "adc.h"
#include "config.h"

#if 0
extern uint16_t ADC_ConvertedValue[9];
extern uint16_t ADC_ConvertedValue2[3];
extern uint16_t ADC_ConvertedValue3[3];
float ADC_ConvertedValueLocal[9]; 

void adc_get_value(void)
{
//    dma_memory_address_config(DMA1, DMA_CH0, DMA_MEMORY_0, (uint32_t)&ADC_ConvertedValue1);
//    dma_transfer_number_config(DMA1, DMA_CH0, 3);
//    dma_channel_enable(DMA1, DMA_CH0);
    ADC_ConvertedValueLocal[0] =(float)(ADC_ConvertedValue[0]*3.3/4096); 
    ADC_ConvertedValueLocal[1] =(float)(ADC_ConvertedValue[1]*3.3/4096);
    ADC_ConvertedValueLocal[2] =(float)(ADC_ConvertedValue[2]*3.3/4096);
//    printf("\r\n 1------------------------- \r\n"); 
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue1[0]); 
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue1[1]);
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue1[2]);
//    dma_memory_address_config(DMA1, DMA_CH1, DMA_MEMORY_0, (uint32_t)&ADC_ConvertedValue2);
//    dma_transfer_number_config(DMA1, DMA_CH1, 3);
//    dma_channel_enable(DMA1, DMA_CH1);
    ADC_ConvertedValueLocal[3] =(float)(ADC_ConvertedValue[3]*3.3/4096); 
    ADC_ConvertedValueLocal[4] =(float)(ADC_ConvertedValue[4]*3.3/4096);
    ADC_ConvertedValueLocal[5] =(float)(ADC_ConvertedValue[5]*3.3/4096);
//    printf("\r\n 2------------------------- \r\n"); 
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue2[0]); 
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue2[1]);
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue2[2]);
//    dma_memory_address_config(DMA1, DMA_CH2, DMA_MEMORY_0, (uint32_t)&ADC_ConvertedValue3);
//    dma_transfer_number_config(DMA1, DMA_CH2, 3);
//    dma_channel_enable(DMA1, DMA_CH2);
    ADC_ConvertedValueLocal[6] =(float)(ADC_ConvertedValue[6]*3.3/4096); 
    ADC_ConvertedValueLocal[7] =(float)(ADC_ConvertedValue[7]*3.3/4096);
    ADC_ConvertedValueLocal[8] =(float)(ADC_ConvertedValue[8]*3.3/4096);
//    printf("\r\n 3------------------------- \r\n"); 
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue3[0]); 
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue3[1]);
//    printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValue3[2]);
    printf("\r\n The current sensor   value = %f V \r\n",ADC_ConvertedValueLocal[0]); 
    printf("\r\n The current 2dcamera value = %f V \r\n",ADC_ConvertedValueLocal[1]);
    printf("\r\n The current vout_12v value = %f V \r\n",ADC_ConvertedValueLocal[2]);
    printf("\r\n The current finger   value = %f V \r\n",ADC_ConvertedValueLocal[3]); 
    printf("\r\n The current 3dcamera value = %f V \r\n",ADC_ConvertedValueLocal[4]);
    printf("\r\n The current light    value = %f V \r\n",ADC_ConvertedValueLocal[5]);
    printf("\r\n The current version  value = %f V \r\n",ADC_ConvertedValueLocal[6]); 
    printf("\r\n The current version1 value = %f V \r\n",ADC_ConvertedValueLocal[7]);
    printf("\r\n The current reserved value = %f V \r\n\r\n",ADC_ConvertedValueLocal[8]);
}

int main(void)
{
    systick_config();
    gpio_config();
    usart3_init(115200);
    uart_dma_init();
    adc_config_init();
    gpio_bit_write(pinList[LIGHT_PSW].port, pinList[LIGHT_PSW].pin, RESET);
    delay_ms(50);
    /* POWER ON CAM 1 */
    gpio_bit_write(pinList[VOUT_12V].port, pinList[VOUT_12V].pin, SET);
    delay_ms(50);
    /* POWER ON 2D CAM*/
    gpio_bit_write(pinList[CAM_2D_PSW].port, pinList[CAM_2D_PSW].pin, SET);
    delay_ms(50);
    /* POWER ON 3D CAM */
    gpio_bit_write(pinList[CAM_3D_PSW].port, pinList[CAM_3D_PSW].pin, SET);
    delay_ms(50);
    /* POWER ON RESERVED 24V */
    gpio_bit_write(pinList[RESERVED_24V].port, pinList[RESERVED_24V].pin, SET);
    delay_ms(50);
    /* POWER ON FINGER */
    gpio_bit_write(pinList[FINGER_PSW].port, pinList[FINGER_PSW].pin, RESET);
    delay_ms(50);
    /* POWER ON SENSOR 24V */
    gpio_bit_write(pinList[SENSOR_PSW].port, pinList[SENSOR_PSW].pin, SET);
    delay_ms(50);
    while(1)
    {
        adc_get_value();
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
        delay_ms(10000);
    }
}
#endif
#if 0
uint16_t adc_value[2];

void rcu_config(void);
void gpio_config_init(void);
void dma_config(void);
void adc_config(void);
void timer_config(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* system clocks configuration */
    gpio_config();
    rcu_config();
    /* GPIO configuration */
    gpio_config_init();
    /* TIMER configuration */
    timer_config();
    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();
    /* configure COM port */
    usart3_init(115200);
    /* configure systick */
    systick_config();
    gpio_bit_write(pinList[LIGHT_PSW].port, pinList[LIGHT_PSW].pin, RESET);
    delay_ms(50);
    /* POWER ON CAM 1 */
    gpio_bit_write(pinList[VOUT_12V].port, pinList[VOUT_12V].pin, SET);
    delay_ms(50);
    /* POWER ON 2D CAM*/
    gpio_bit_write(pinList[CAM_2D_PSW].port, pinList[CAM_2D_PSW].pin, SET);
    delay_ms(50);
    /* POWER ON 3D CAM */
    gpio_bit_write(pinList[CAM_3D_PSW].port, pinList[CAM_3D_PSW].pin, SET);
    delay_ms(50);
    /* POWER ON RESERVED 24V */
    gpio_bit_write(pinList[RESERVED_24V].port, pinList[RESERVED_24V].pin, SET);
    delay_ms(50);
    /* POWER ON FINGER */
    gpio_bit_write(pinList[FINGER_PSW].port, pinList[FINGER_PSW].pin, RESET);
    delay_ms(50);
    /* POWER ON SENSOR 24V */
    gpio_bit_write(pinList[SENSOR_PSW].port, pinList[SENSOR_PSW].pin, RESET);
    delay_ms(50);

    while(1){
        delay_ms(500);
        printf(" the data adc_value[0] is %08X \r\n",adc_value[0]);
        printf(" the data adc_value[1] is %08X \r\n",adc_value[1]);
        printf("\r\n The current ADC1 value = %f V \r\n",(float)(adc_value[0]*3.3/4096)); 
        printf("\r\n The current ADC2 value = %f V \r\n",(float)(adc_value[1]*3.3/4096));
        printf("\r\n");
    }
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOC clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA1);
    /* enable TIMER1 clock */
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* config ADC clock */
    adc_clock_config(ADC_ADCCK_PCLK2_DIV4);
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config_init(void)
{
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_4);
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_6);
}

/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_single_data_parameter_struct dma_single_data_parameter;
    
    /* ADC DMA_channel deinit */
    dma_deinit(DMA1,DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_single_data_parameter.periph_addr = (uint32_t)(&ADC_SYNCDATA);
    dma_single_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_single_data_parameter.memory0_addr = (uint32_t)(&adc_value);
    dma_single_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_single_data_parameter.periph_memory_width = DMA_PERIPH_WIDTH_32BIT;
    dma_single_data_parameter.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_single_data_parameter.direction = DMA_PERIPH_TO_MEMORY;
    dma_single_data_parameter.number = 2;
    dma_single_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA1,DMA_CH0, &dma_single_data_parameter);
    /* DMA channel 0 peripheral select */
    dma_channel_subperipheral_select(DMA1,DMA_CH0,DMA_SUBPERI0);

    /* enable DMA channel */
    dma_channel_enable(DMA1,DMA_CH0);
}

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* ADC channel length config */
    adc_channel_length_config(ADC0,ADC_REGULAR_CHANNEL,2);
    adc_channel_length_config(ADC1,ADC_REGULAR_CHANNEL,2);
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0,0,ADC_CHANNEL_4,ADC_SAMPLETIME_3);
    adc_regular_channel_config(ADC0,1,ADC_CHANNEL_6,ADC_SAMPLETIME_3);
    adc_regular_channel_config(ADC1,1,ADC_CHANNEL_4,ADC_SAMPLETIME_3);
    adc_regular_channel_config(ADC1,0,ADC_CHANNEL_6,ADC_SAMPLETIME_3);
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0,ADC_REGULAR_CHANNEL,EXTERNAL_TRIGGER_RISING);
    adc_external_trigger_config(ADC1,ADC_REGULAR_CHANNEL,EXTERNAL_TRIGGER_DISABLE);
    adc_external_trigger_source_config(ADC0,ADC_REGULAR_CHANNEL,ADC_EXTTRIG_REGULAR_T1_CH1);
    
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1,ADC_DATAALIGN_RIGHT);
    /* configure the ADC sync mode */
    adc_sync_mode_config(ADC_DAUL_REGULAL_PARALLEL);
    adc_sync_dma_config(ADC_SYNC_DMA_MODE0);
    adc_sync_dma_request_after_last_enable();
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0,ADC_SCAN_MODE,ENABLE);
    adc_special_function_config(ADC1,ADC_SCAN_MODE,ENABLE);
    adc_resolution_config(ADC0,ADC_RESOLUTION_12B);
    adc_resolution_config(ADC1,ADC_RESOLUTION_12B);
    adc_special_function_config(ADC0,ADC_CONTINUOUS_MODE,ENABLE);
    adc_special_function_config(ADC1,ADC_CONTINUOUS_MODE,ENABLE);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    /* enable ADC interface */
    adc_enable(ADC1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
}

/*!
    \brief      configure the timer peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer_config(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 19999;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH1 configuration in PWM mode0 */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,3999);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
    
    /* enable TIMER1 */
    timer_enable(TIMER1);
}
#endif


#if 1
float ADC_ConvertedValueLocal[9]; 

void adc_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_ADC0);
    adc_clock_config(ADC_ADCCK_PCLK2_DIV4);

    /*
    A3 ADC012_IN3
    A4 ADC01_IN4 2.0
    A5 ADC01_IN5
    A6 ADC01_IN6
    B0 ADC01_IN8
    B1 ADC01_IN9
    A0 ADC012_IN0
    C0 ADC012_IN10
    C3 ADC012_IN13
    */
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_0);
    gpio_mode_set(GPIOC,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_0);
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_3);
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_4);
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_5);
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_6);
    gpio_mode_set(GPIOB,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_0);
    gpio_mode_set(GPIOB,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_1);
    gpio_mode_set(GPIOC,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_3);

    //设置规则采样
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);

    //设置右对齐
    adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
    //设置通道数量
    adc_channel_length_config(ADC0,ADC_REGULAR_CHANNEL,9);
    //设置分辨率
    adc_resolution_config(ADC0,ADC_RESOLUTION_12B);
    //设置不外部触发
    adc_external_trigger_config(ADC0,ADC_REGULAR_CHANNEL,EXTERNAL_TRIGGER_DISABLE);
    adc_enable(ADC0);
    delay_ms(2);
    adc_calibration_enable(ADC0);
}


uint16_t get_adc_channel_value(uint8_t channel)
{
    adc_regular_channel_config(ADC0, 0, channel, ADC_SAMPLETIME_112);
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    return adc_regular_data_read(ADC0);
}

int main(void)
{
    systick_config();
    gpio_config();
    adc_config();
    usart3_init(115200);
    gpio_bit_write(pinList[LIGHT_PSW].port, pinList[LIGHT_PSW].pin, RESET);
    delay_ms(50);
    /* POWER ON CAM 1 */
    gpio_bit_write(pinList[VOUT_12V].port, pinList[VOUT_12V].pin, SET);
    delay_ms(50);
    /* POWER ON 2D CAM*/
    gpio_bit_write(pinList[CAM_2D_PSW].port, pinList[CAM_2D_PSW].pin, SET);
    delay_ms(50);
    /* POWER ON 3D CAM */
    gpio_bit_write(pinList[CAM_3D_PSW].port, pinList[CAM_3D_PSW].pin, SET);
    delay_ms(50);
    /* POWER ON RESERVED 24V */
    gpio_bit_write(pinList[RESERVED_24V].port, pinList[RESERVED_24V].pin, SET);
    delay_ms(50);
    /* POWER ON FINGER */
    gpio_bit_write(pinList[FINGER_PSW].port, pinList[FINGER_PSW].pin, RESET);
    delay_ms(50);
    /* POWER ON SENSOR 24V */
    gpio_bit_write(pinList[SENSOR_PSW].port, pinList[SENSOR_PSW].pin, SET);
    delay_ms(50);

    while(1)
    {
        /* delay a time in milliseconds */
        delay_ms(2000);
        ADC_ConvertedValueLocal[0] = (get_adc_channel_value(ADC_CHANNEL_3)*3.3/4096);
        ADC_ConvertedValueLocal[1] = (get_adc_channel_value(ADC_CHANNEL_4)*3.3/4096);
        ADC_ConvertedValueLocal[2] = (get_adc_channel_value(ADC_CHANNEL_5)*3.3/4096);
        ADC_ConvertedValueLocal[3] = (get_adc_channel_value(ADC_CHANNEL_6)*3.3/4096);
        ADC_ConvertedValueLocal[4] = (get_adc_channel_value(ADC_CHANNEL_8)*3.3/4096);
        ADC_ConvertedValueLocal[5] = (get_adc_channel_value(ADC_CHANNEL_9)*3.3/4096);
        ADC_ConvertedValueLocal[6] = (get_adc_channel_value(ADC_CHANNEL_0)*3.3/4096);
        ADC_ConvertedValueLocal[7] = (get_adc_channel_value(ADC_CHANNEL_10)*3.3/4096);
        ADC_ConvertedValueLocal[8] = (get_adc_channel_value(ADC_CHANNEL_13)*3.3/4096);
        printf("\r\n The current sensor   value = %f V \r\n",ADC_ConvertedValueLocal[0]); 
        printf("\r\n The current 2dcamera value = %f V \r\n",ADC_ConvertedValueLocal[1]);
        printf("\r\n The current vout_12v value = %f V \r\n",ADC_ConvertedValueLocal[2]);
        printf("\r\n The current finger   value = %f V \r\n",ADC_ConvertedValueLocal[3]); 
        printf("\r\n The current 3dcamera value = %f V \r\n",ADC_ConvertedValueLocal[4]);
        printf("\r\n The current light    value = %f V \r\n",ADC_ConvertedValueLocal[5]);
        printf("\r\n The current version  value = %f V \r\n",ADC_ConvertedValueLocal[6]); 
        printf("\r\n The current version1 value = %f V \r\n",ADC_ConvertedValueLocal[7]);
        printf("\r\n The current reserved value = %f V \r\n",ADC_ConvertedValueLocal[8]);
        printf("\r\n");
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
    }
}
#endif
int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART3, (uint32_t)ch);
    while(!usart_flag_get(UART3, USART_FLAG_TBE));
    return ch;
}

