#include "delay.h"

#define DELAY_TIMER_RCU_PERIPH  RCU_TIMER1
#define DELAY_TIMER_PERIPH      TIMER1
#define DELAY_TIMER_NVIC_IRQ    TIMER1_IRQn
#define DELAY_TIMER_IRQ_HANDLER TIMER1_IRQHandler

//int_freq = CLK / ((prescaler + 1) * period)
//int_freq = 1us
#define DELAY_TIMER_PRESCALER   1
#define DELAY_TIMER_PERIOD      41999

volatile static uint32_t gs_count = 0;

void time1_init(void)
{
    timer_parameter_struct timer_parameter;

    rcu_periph_clock_enable(DELAY_TIMER_RCU_PERIPH);
    timer_parameter.prescaler = DELAY_TIMER_PRESCALER;
    timer_parameter.alignedmode = TIMER_COUNTER_EDGE;
    timer_parameter.counterdirection = TIMER_COUNTER_UP;
    timer_parameter.period = DELAY_TIMER_PERIOD;
    timer_parameter.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(DELAY_TIMER_PERIPH, &timer_parameter);
    timer_auto_reload_shadow_enable(DELAY_TIMER_PERIPH);
    timer_enable(DELAY_TIMER_PERIPH);
    timer_interrupt_flag_clear(DELAY_TIMER_PERIPH, TIMER_INT_UP);
    timer_interrupt_enable(DELAY_TIMER_PERIPH, TIMER_INT_UP);
    nvic_irq_enable(DELAY_TIMER_NVIC_IRQ, 0, 0);
}

void DELAY_TIMER_IRQ_HANDLER(void)
{
    if(SET == timer_interrupt_flag_get(DELAY_TIMER_PERIPH, TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(DELAY_TIMER_PERIPH, TIMER_INT_UP);
        gs_count++;
    }
}

void measure_runtime_start(void)
{
     gs_count = 0;
     timer_enable(DELAY_TIMER_PERIPH);
}

float measure_runtime_end(void)
{
    float work_time;
    uint32_t cnt_num,tmp;
    cnt_num = TIMER_CNT(DELAY_TIMER_PERIPH);
    timer_disable(DELAY_TIMER_PERIPH);
    nvic_irq_disable(DELAY_TIMER_NVIC_IRQ);
    tmp = gs_count*10000 + cnt_num;
    work_time = (float)tmp/1000.0;
    return work_time;
}

void time1_delay_us(uint32_t us)
{
    uint32_t start_count = gs_count;
    while((gs_count - start_count) < us);
}

void time1_delay_ms(uint32_t ms)
{
    time1_delay_us(ms * 1000);
}
