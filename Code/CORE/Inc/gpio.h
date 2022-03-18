#ifndef __GPIO_H_
#define __GPIO_H_

#include "gd32f4xx.h"

typedef struct
{
    uint32_t port;                       /**< 端口 */
    uint32_t pin;                               /**< 引脚 */
    uint32_t mode;                              /**< 模式 */
    uint32_t pull;                              /**< 上下拉 */
    uint32_t speed;                             /**< 速度 */
    uint8_t otype;                              /**< 引脚复用 */
} PinDef;

enum
{
    POS1,
    POS2,
    POS3,
    POS4,
    POS5,
    POS6,
    POS7,
    POS8,
    LED_RUN,
    FINGER_PSW,
    FORK_CAM_PSW_1,
    FORK_CAM_PSW_2,
    FINGER_24V_PSW,
    PHY_RST,
    LIGHT_PSW_1,
    LIGHT_PSW_2,
    CAM_3D_LIGH_PSW,
    RFID_PSW,
    SWITCHES_PSW,
    LED_DEBUG_1,
    LED_DEBUG_2,
    LIGH_PSW_REV,
    FAN_STATE_DET,
    FAN_PSW,
    FINGER_24V_DET,
    POW_DET,
};

#define PIN_DEFINE(port, pin, mode, pull, speed, otype) \
        {GPIO##port, GPIO_PIN_##pin, mode, pull, speed, otype}

#define gd_led_toggle(port, pin) GPIO_TG(port) = pin;

extern const PinDef pinList[];
void gpio_config(void);

#endif
