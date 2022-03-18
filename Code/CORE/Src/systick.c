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

static uint8_t  fac_us=0;                 //us��ʱ������
static uint16_t fac_ms=0;                 //ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

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
    uint32_t reload=SysTick->LOAD;            //LOAD��ֵ
    ticks=nus*fac_us;                         //��Ҫ�Ľ����� 
    told=SysTick->VAL;                        //�ս���ʱ�ļ�����ֵ
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)
              tcnt+=told-tnow;                //����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
            else
              tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)
              break;                          //ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
        }
    }
}
 
 
void delay_ms(uint32_t nms)
{
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) //ϵͳ�Ѿ�����
    {
        if(nms>=fac_ms)                                     //��ʱ��ʱ�����OS������ʱ������
        {
            vTaskDelay(nms/fac_ms);                         //FreeRTOS��ʱ
        }
        nms%=fac_ms;                                        //OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ
    }
    delay_us((uint32_t)(nms*1000));                         //��ͨ��ʽ��ʱ
}

#else
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD=nus*fac_us;                 //ʱ�����
    SysTick->VAL=0x00;                        //��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  //��ʼ����
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));     //�ȴ�ʱ�䵽��
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;  //�رռ�����
    SysTick->VAL =0X00;                       //��ռ����� 
}


void delay_xms(uint16_t nms)
{
    uint32_t temp;
    SysTick->LOAD=(uint32_t)nms*fac_ms;       //ʱ�����(SysTick->LOADΪ24bit)
    SysTick->VAL =0x00;                       //��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  //��ʼ����
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));     //�ȴ�ʱ�䵽��
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;  //�رռ�����
    SysTick->VAL =0X00;                       //��ռ�����
} 


void delay_ms(uint16_t nms)
{
    uint8_t repeat=nms/540;                 //������540,�ǿ��ǵ�ĳЩ�ͻ����ܳ�Ƶʹ��,���糬Ƶ��248M��ʱ��,delay_xms���ֻ����ʱ541ms������
    uint16_t remain=nms%540;
    while(repeat)
    {
        delay_xms(540);
        repeat--;
    }
    if(remain)delay_xms(remain);
} 
#endif
