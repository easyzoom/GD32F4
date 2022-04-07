#include "iic.h"


void i2c_config(void)
{
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_8);
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_9);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    i2c_clock_config(I2C0,high_speed,I2C_DTCY_2);
    i2c_mode_addr_config(I2C0,I2C_I2CMODE_ENABLE,I2C_ADDFORMAT_7BITS,I2C0_SLAVE_ADDRESS);
    i2c_enable(I2C0);
    i2c_ack_config(I2C0,I2C_ACK_ENABLE);
}

