#ifndef __PCA9535PW_H_
#define __PCA9535PW_H_

#include "gd32f4xx.h"

#define PCA9535_ADDR    (0x4E)
#define WRITE_BIT       (0x00)
#define READ_BIT        (0x01)

#define PCA9535_CONFIG_INPUT_VAL    (0xFF)
#define PCA9535_CONFIG_OUTPUT_VAL   (0x00)

#
typedef enum {
    PCA9535_INPUT_REG0 =    0x00,
    PCA9535_INPUT_REG1 =    0x01,
    PCA9535_OUTPUT_REG0 =   0x02,
    PCA9535_OUTPUT_REG1 =   0x03,
    PCA9535_POLARITY_REG0 = 0x04,
    PCA9535_POLARITY_REG1 = 0x05,
    PCA9535_CONFIG_REG0 =   0x06,
    PCA9535_CONFIG_REG1 =   0x07
} pca9535_reg_t;

void pca9535_init(void);
int pca9535_write(pca9535_reg_t address, uint8_t regvalue);
int pca9535_read(pca9535_reg_t address, uint8_t *readbyte);
void test_pca9535(void);
#endif
