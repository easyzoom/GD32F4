#include "pca9535pw.h"
#include "iic.h"
#include <stdio.h>


void pca9535_init(void)
{
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    i2c_start_on_bus(I2C0);
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    i2c_master_addressing(I2C0, PCA9535_ADDR, I2C_TRANSMITTER);
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0,I2C_FLAG_ADDSEND);
    while( SET != i2c_flag_get(I2C0, I2C_FLAG_TBE));
}

int pca9535_write(pca9535_reg_t address, uint8_t regvalue)
{
    uint8_t err = 0;
    i2c_start_on_bus(I2C0);
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    i2c_master_addressing(I2C0, PCA9535_ADDR | WRITE_BIT, I2C_TRANSMITTER);
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    i2c_data_transmit(I2C0, address);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    i2c_data_transmit(I2C0, regvalue);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    i2c_stop_on_bus(I2C0);
    return err;
}

int pca9535_read(pca9535_reg_t address, uint8_t *readbyte)
{
    uint8_t err = 1;
    i2c_start_on_bus(I2C0);
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    i2c_master_addressing(I2C0, PCA9535_ADDR | WRITE_BIT, I2C_TRANSMITTER);
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    i2c_data_transmit(I2C0, address);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    i2c_start_on_bus(I2C0);
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    i2c_master_addressing(I2C0, PCA9535_ADDR | WRITE_BIT, I2C_RECEIVER);
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    i2c_stop_on_bus(I2C0);
    while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    *readbyte  = i2c_data_receive(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    err = 0;
    return err;
}


void test_pca9535(void)
{
    uint8_t test_date;
#if 0
    pca9535_write(PCA9535_OUTPUT_REG0, 0x00);
    pca9535_write(PCA9535_OUTPUT_REG1, 0x00);
    pca9535_write(PCA9535_CONFIG_REG0, PCA9535_CONFIG_OUTPUT_VAL);
    pca9535_write(PCA9535_CONFIG_REG0, PCA9535_CONFIG_OUTPUT_VAL);
#endif

#if 1
    pca9535_write(PCA9535_CONFIG_REG0, PCA9535_CONFIG_INPUT_VAL);
    pca9535_write(PCA9535_CONFIG_REG1, PCA9535_CONFIG_INPUT_VAL);
    pca9535_read(PCA9535_INPUT_REG0, &test_date);
    printf("1111test_date: 0x%02x\r\n", test_date);
    pca9535_read(PCA9535_INPUT_REG1, &test_date);
    printf("2222test_date: 0x%02x\r\n", test_date);
#endif
}
