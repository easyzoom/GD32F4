#include "adc.h"
#include "systick.h"

void adc_config_init(void)
{
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);
    adc_clock_config(ADC_ADCCK_PCLK2_DIV8);

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

    adc_deinit();
    //设置规则采样
    adc_sync_mode_config(ADC_ALL_REGULAL_PARALLEL);
    //ADC的DMA模式
    adc_sync_dma_config(ADC_SYNC_DMA_MODE0);
    adc_sync_dma_request_after_last_disable();

    //设置扫描模式
    adc_special_function_config(ADC0,ADC_SCAN_MODE,ENABLE);
    adc_special_function_config(ADC1,ADC_SCAN_MODE,ENABLE);
    adc_special_function_config(ADC2,ADC_SCAN_MODE,ENABLE);
    //设置连续模式
    adc_special_function_config(ADC0,ADC_CONTINUOUS_MODE,ENABLE);
    adc_special_function_config(ADC1,ADC_CONTINUOUS_MODE,ENABLE);
    adc_special_function_config(ADC2,ADC_CONTINUOUS_MODE,ENABLE);
    //设置右对齐
    adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1,ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC2,ADC_DATAALIGN_RIGHT);

    //设置通道数量
    adc_channel_length_config(ADC0,ADC_REGULAR_CHANNEL,3);
    adc_channel_length_config(ADC1,ADC_REGULAR_CHANNEL,3);
    adc_channel_length_config(ADC2,ADC_REGULAR_CHANNEL,3);
    //设置通道采样率
    adc_regular_channel_config(ADC0,0,ADC_CHANNEL_3,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC0,1,ADC_CHANNEL_4,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC0,2,ADC_CHANNEL_5,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC1,0,ADC_CHANNEL_6,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC1,1,ADC_CHANNEL_8,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC1,2,ADC_CHANNEL_9,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC2,0,ADC_CHANNEL_0,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC2,1,ADC_CHANNEL_10,ADC_SAMPLETIME_144);
    adc_regular_channel_config(ADC2,2,ADC_CHANNEL_13,ADC_SAMPLETIME_144);
    //设置分辨率
    adc_resolution_config(ADC0,ADC_RESOLUTION_12B);
    adc_resolution_config(ADC1,ADC_RESOLUTION_12B);
    adc_resolution_config(ADC2,ADC_RESOLUTION_12B);
    //设置不外部触发
    adc_external_trigger_config(ADC0,ADC_REGULAR_CHANNEL,EXTERNAL_TRIGGER_DISABLE);
    adc_external_trigger_config(ADC1,ADC_REGULAR_CHANNEL,EXTERNAL_TRIGGER_DISABLE);
    adc_external_trigger_config(ADC2,ADC_REGULAR_CHANNEL,EXTERNAL_TRIGGER_DISABLE);
    
    adc_enable(ADC0);
    delay_ms(2);
    //校准使能
    adc_calibration_enable(ADC0);
    adc_enable(ADC1);
    delay_ms(2);
    adc_calibration_enable(ADC1);
    adc_enable(ADC2);
    delay_ms(2);
    adc_calibration_enable(ADC2);
    adc_dma_request_after_last_enable(ADC0);
    adc_dma_request_after_last_enable(ADC1);
    adc_dma_request_after_last_enable(ADC2);
//    //使能adc dma模式
    adc_dma_mode_enable(ADC0);
//    adc_dma_mode_enable(ADC1);
//    adc_dma_mode_enable(ADC2);
    //设置软件触发
    adc_software_trigger_enable(ADC0,ADC_REGULAR_CHANNEL);
    adc_software_trigger_enable(ADC1,ADC_REGULAR_CHANNEL);
    adc_software_trigger_enable(ADC2,ADC_REGULAR_CHANNEL);

}

