#include "gpio.h"

const PinDef pinList[] = 
{
    [LED_RUN]   = PIN_DEFINE(E, 10, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [USART_REN] = PIN_DEFINE(A, 11, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [SW_RESET]      = PIN_DEFINE(C, 13, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [FINGER_PSW]    = PIN_DEFINE(E, 0, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [FINGER_CHECK]  = PIN_DEFINE(E, 1, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [CAM_2D_CHECK]  = PIN_DEFINE(E, 2, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [SENSOR_PSW]    = PIN_DEFINE(E, 3, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [SENSOR_CHECK]  = PIN_DEFINE(E, 4, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [VOUT_12V]      = PIN_DEFINE(E, 5, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [CAM_2D_PSW]    = PIN_DEFINE(E, 6, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [CAM_3D_PSW]    = PIN_DEFINE(E, 7, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [RESERVED_24V]  = PIN_DEFINE(E, 8, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [CAM_3D_CHECK]  = PIN_DEFINE(E, 8, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [LIGHT_PSW]     = PIN_DEFINE(E, 10, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [ROTATE_LEFT]   = PIN_DEFINE(B, 10, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [ROTATE_RIGHT]  = PIN_DEFINE(D, 7, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
    [STRETCH_ZERO]  = PIN_DEFINE(B, 14, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP),
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

