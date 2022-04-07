#ifndef __IIC_H_
#define __IIC_H_

#include "gd32f4xx.h"


typedef enum
{
    standard_speed    = 100000,
    high_speed        = 400000,
    more_high_speed   = 1000000
}iic_speed_enum;


#define I2C0_SLAVE_ADDRESS     0x4E

void i2c_config(void);
#endif
