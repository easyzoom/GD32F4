#include "can.h"

void can_config_init(void)
{
    can_parameter_struct can_parameter;
    rcu_periph_clock_enable(RCU_CAN0);
    gpio_af_set(GPIOD, GPIO_AF_9, GPIO_PIN_0);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_af_set(GPIOD, GPIO_AF_9, GPIO_PIN_1);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    can_deinit(CAN0);
    can_parameter.working_mode = CAN_LOOPBACK_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = ENABLE;
    can_parameter.no_auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.prescaler = 10;
    can_init(CAN0, &can_parameter);

    rcu_periph_clock_enable(RCU_CAN1);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_6);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    can_deinit(CAN1);
    can_parameter.working_mode = CAN_LOOPBACK_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = ENABLE;
    can_parameter.no_auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.prescaler = 10;
    can_init(CAN1, &can_parameter);
}


void can_filter_config_init(void)
{
    can_filter_parameter_struct can0_filter_parameter;
    can0_filter_parameter.filter_list_high = 0x0000U;
    can0_filter_parameter.filter_list_low = 0x0000U;
    can0_filter_parameter.filter_mask_high = 0x0000U;
    can0_filter_parameter.filter_mask_low = 0x0000U;
    can0_filter_parameter.filter_fifo_number = CAN_FIFO0;
    can0_filter_parameter.filter_number = 0;
    can0_filter_parameter.filter_mode = CAN_FILTERMODE_MASK;
    can0_filter_parameter.filter_bits = CAN_FILTERBITS_32BIT;
    can0_filter_parameter.filter_enable = DISABLE;
    can_filter_init(&can0_filter_parameter);
//    can_interrupt_enable(CAN0, CAN_INT_RFNE0);
    nvic_irq_enable(CAN0_RX0_IRQn,0,0);

    can_filter_parameter_struct can1_filter_parameter;
    can1_filter_parameter.filter_list_high = 0x0000U;
    can1_filter_parameter.filter_list_low = 0x0000U;
    can1_filter_parameter.filter_mask_high = 0x0000U;
    can1_filter_parameter.filter_mask_low = 0x0000U;
    can1_filter_parameter.filter_fifo_number = CAN_FIFO1;
    can1_filter_parameter.filter_number = 15;
    can1_filter_parameter.filter_mode = CAN_FILTERMODE_MASK;
    can1_filter_parameter.filter_bits = CAN_FILTERBITS_32BIT;
    can1_filter_parameter.filter_enable = DISABLE;
    can_filter_init(&can1_filter_parameter);
//    can_interrupt_enable(CAN1, CAN_INT_RFNE1);
    nvic_irq_enable(CAN1_RX0_IRQn,0,0);
}

uint32_t can_get_txmailboxfreelevel(uint32_t can_periph)
{
    uint32_t freelevel = 0U;
 
    for(int i = 0; i <= 2; i++)
    {
        if(CAN_TRANSMIT_PENDING == can_transmit_states(can_periph, i))
        {
            freelevel++;
            break;
        }
    }

    return freelevel;
}

