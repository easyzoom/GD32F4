#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "config.h"
#include "delay.h"
#include "cmsis_os.h"

#include "kubot_debug.h"
#include "httpd.h"
#include "delay.h"
#include "fs_api.h"
#include "sys_info.h"
#include "switch_app.h"
#include "ethernetif.h"
#include "lwip.h"
#include "httpd_cgi_ssi.h"
uint16_t led_debug_1_tick = 500;
uint8_t reboot_flag = 0;
uint16_t reboot_timeout = 0;

/**
 * @brief ϵͳ��������ʱ
 * 
 * @param [in] time �����ϵͳ������0��Ч
 * @return void
 */
void set_reboot(uint16_t time)
{
    reboot_timeout = time;
}

/**
 * @brief ϵͳ��������
 * 
 * @param void
 * @return void
 */
void reboot(void)
{
    static uint16_t time = 0;
    if(reboot_timeout)
    {
        if(time++ > reboot_timeout)reboot_flag = 1;
    }
    if(reboot_flag)
    {
        taskENTER_CRITICAL();  /* �����ٽ��� */
        NVIC_SystemReset(); // ��λ
    }
}

/**
 * @brief ϵͳ�ָ�Ĭ������
 * 
 * @param void
 * @return 
 */
int sys_default(void)
{
    int state = 0;
    
    state = fs_api_format();
    if(state)
    {
        LOG_PRINT_ERROR("File system format error\r\n");    
    }
    set_reboot(200);
    return state;
}

/**
 * @brief ϵͳָʾ��
 * 
 * @param void
 * @return void
 */
void led_ctrl(void)
{
    if (xTaskGetTickCount() >= led_debug_1_tick + 500)
    {
        gd_led_toggle(pinList[LED_RUN].port, pinList[LED_RUN].pin);
        led_debug_1_tick = xTaskGetTickCount();
    }
}

int main(void)
{
    
    uint8_t state = 0;
//    systick_config();
    gpio_config();
    gpio_bit_write(pinList[SW_RESET].port, pinList[SW_RESET].pin, SET);
    time_delay_init();
    usart3_init(115200);
    enet_system_setup();
    state = fs_api_init();
    sys_info_config(state);
    printf("HR-LINK system starting...\r\n");
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    MX_FREERTOS_Init();
    osKernelStart();
    while(1)
    {
    }
}

void StartDefaultTask(void const * argument)
{
    lwip_stack_init();
    switch_cfg();
//    hello_gigadevice_init();
//    tcp_client_init();
//    udp_echo_init();
    
    httpd_init();
    httpd_ssi_init();
    httpd_cgi_init();
    for (;;)
    {
        web_login_monitor();
        reboot();
        switch_app();
        led_ctrl();
        osDelay(1);
    }
}
