/*!
    \file    systick.c
    \brief   the systick configuration file
    
    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#include "cmsis_os.h"

#define SYSTEM_SUPPORT_OS   1

static uint8_t  fac_us=0;                 //us延时倍乘数
static uint16_t fac_ms=0;                 //ms延时倍乘数,在ucos下,代表每个节拍的ms数

void systick_config(void)
{
#if SYSTEM_SUPPORT_OS
    uint32_t reload;
#endif
    systick_clksource_set(SYSTICK_CLKSOURCE_HCLK);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
    fac_us=SystemCoreClock/1000000;
#if SYSTEM_SUPPORT_OS
    reload=SystemCoreClock/1000000;
    reload*=1000000/configTICK_RATE_HZ;
    fac_ms=1000/configTICK_RATE_HZ;
 
    SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD=reload;
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
#else
    fac_ms=(uint16_t)fac_us*1000;
#endif
}

#if SYSTEM_SUPPORT_OS
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;            //LOAD的值
    ticks=nus*fac_us;                         //需要的节拍数 
    told=SysTick->VAL;                        //刚进入时的计数器值
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)
              tcnt+=told-tnow;                //这里注意一下SYSTICK是一个递减的计数器就可以了.
            else
              tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)
              break;                          //时间超过/等于要延迟的时间,则退出.
        }
    }
}
 
 
void delay_ms(uint32_t nms)
{
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) //系统已经运行
    {
        if(nms>=fac_ms)                                     //延时的时间大于OS的最少时间周期
        {
            vTaskDelay(nms/fac_ms);                         //FreeRTOS延时
        }
        nms%=fac_ms;                                        //OS已经无法提供这么小的延时了,采用普通方式延时
    }
    delay_us((uint32_t)(nms*1000));                         //普通方式延时
}

#else
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD=nus*fac_us;                 //时间加载
    SysTick->VAL=0x00;                        //清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  //开始倒数
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));     //等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;  //关闭计数器
    SysTick->VAL =0X00;                       //清空计数器 
}


void delay_xms(uint16_t nms)
{
    uint32_t temp;
    SysTick->LOAD=(uint32_t)nms*fac_ms;       //时间加载(SysTick->LOAD为24bit)
    SysTick->VAL =0x00;                       //清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  //开始倒数
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));     //等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;  //关闭计数器
    SysTick->VAL =0X00;                       //清空计数器
} 


void delay_ms(uint16_t nms)
{
    uint8_t repeat=nms/540;                 //这里用540,是考虑到某些客户可能超频使用,比如超频到248M的时候,delay_xms最大只能延时541ms左右了
    uint16_t remain=nms%540;
    while(repeat)
    {
        delay_xms(540);
        repeat--;
    }
    if(remain)delay_xms(remain);
} 
#endif
