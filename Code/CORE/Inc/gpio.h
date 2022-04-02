#ifndef __GPIO_H_
#define __GPIO_H_

#include "gd32f4xx.h"

typedef struct
{
    uint32_t port;                       /**< �˿� */
    uint32_t pin;                               /**< ���� */
    uint32_t mode;                              /**< ģʽ */
    uint32_t pull;                              /**< ������ */
    uint32_t speed;                             /**< �ٶ� */
    uint8_t otype;                              /**< ���Ÿ��� */
} PinDef;

enum
{
    LED_RUN,
    USART_REN,
    SW_RESET,
    FINGER_PSW,
    FINGER_CHECK,
    CAM_2D_CHECK,
    SENSOR_PSW,
    SENSOR_CHECK,
    VOUT_12V,
    CAM_2D_PSW,
    CAM_3D_PSW,
    RESERVED_24V,
    CAM_3D_CHECK,
    LIGHT_PSW,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    STRETCH_ZERO,
};

#define PIN_DEFINE(port, pin, mode, pull, speed, otype) \
        {GPIO##port, GPIO_PIN_##pin, mode, pull, speed, otype}

#define gd_led_toggle(port, pin) GPIO_TG(port) = pin;

extern const PinDef pinList[];
void gpio_config(void);

#endif
