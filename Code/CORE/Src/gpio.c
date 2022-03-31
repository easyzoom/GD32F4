#include "gpio.h"

const PinDef pinList[] = 
{
    [LED_RUN]   = PIN_DEFINE(E, 10, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [USART_REN] = PIN_DEFINE(A, 11, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
};

static void gpio_clk_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOI);
}

void gpio_config(void)
{
    gpio_clk_config();
    for(int i = 0; i < sizeof(pinList)/sizeof(PinDef); i++)
    {
        gpio_mode_set(pinList[i].port, pinList[i].mode, pinList[i].pull, pinList[i].pin);
        gpio_output_options_set(pinList[i].port, pinList[i].otype, pinList[i].speed, pinList[i].pin);
        gpio_bit_reset(pinList[i].port, pinList[i].pin);
    }
}

