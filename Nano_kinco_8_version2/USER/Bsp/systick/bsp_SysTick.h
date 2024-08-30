#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f4xx.h"

#define USECS_PER_HIGH     1000000000
#define USECS_PER_MSEC    1000


typedef struct
{
    __IO u32 usecH;
    __IO u32 usecL;
} sys_timer_t;

//�ڵδ�ʱ���жϷ������е���
void TimingDelay_Decrement(void);

// ��ʼ��ϵͳ�δ�ʱ��
void SysTick_Init(void);


//�ṩ��Ӧ�ó������
void Delay_ms(__IO u32 nTime);
#define Delay_10ms(x) Delay_ms(10*x)

uint64_t osal_getSysTime_us(void);

#endif /* __SYSTICK_H */
