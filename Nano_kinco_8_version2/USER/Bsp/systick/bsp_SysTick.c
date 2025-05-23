//############################################################
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//EtherCAT从站学习板
//Author：廷华电子设计
//淘宝店铺: https://shop461235811.taobao.com/
//我的博客：https://blog.csdn.net/zhandouhu/article/category/9455918
//############################################################
  
#include "./Bsp/systick/bsp_SysTick.h"
#include "./soem/osal.h"
static __IO uint32_t TimingDelay;
sys_timer_t sys_timer = {0};

__IO int32_t TimeOut = 0; 

void Delay_us(__IO u32 nTime);

#define  timercmp(a, b, CMP)                                \
  (((a)->usecH == (b)->usecH) ?                           \
   ((a)->usecL CMP (b)->usecL) :                        \
   ((a)->usecH CMP (b)->usecH))
#define  timeradd(a, b, result)                             \
  do {                                                      \
    (result)->usecH = (a)->usecH + (b)->usecH;           \
    (result)->usecL = (a)->usecL + (b)->usecL;        \
    if ((result)->usecL >= USECS_PER_HIGH)                       \
    {                                                       \
       ++(result)->usecH;                                  \
       (result)->usecL -= USECS_PER_HIGH;                        \
    }                                                       \
  } while (0)
#define  timersub(a, b, result)                             \
  do {                                                      \
    (result)->usecH = (a)->usecH - (b)->usecH;           \
    (result)->usecL = (a)->usecL - (b)->usecL;        \
    if ((result)->usecL < 0) {                            \
      --(result)->usecH;                                   \
      (result)->usecL += USECS_PER_HIGH;                         \
    }                                                       \
  } while (0)


void osal_getSysTime (sys_timer_t *timer)
{
    timer->usecH =  sys_timer.usecH;
	timer->usecL =  sys_timer.usecL;
}

uint64_t osal_getSysTime_us(void)
{
	return sys_timer.usecH * USECS_PER_HIGH + sys_timer.usecL;
}

void osal_timer_start (osal_timert * self,uint32_t timeout_usec)
{  
   sys_timer_t start_time;
   sys_timer_t timeout;
   sys_timer_t stop_time;

   osal_getSysTime (&start_time);
   
   timeout.usecH = timeout_usec / USECS_PER_HIGH;
   timeout.usecL = timeout_usec % USECS_PER_HIGH;
   timeradd (&start_time, &timeout, &stop_time);

   self->stop_time.usecH = stop_time.usecH;
   self->stop_time.usecL = stop_time.usecL;
}

boolean osal_timer_is_expired (osal_timert * self)
{  
    sys_timer_t current_time;
    sys_timer_t stop_time;
    int is_not_yet_expired;

    osal_getSysTime (&current_time);
    stop_time.usecH = self->stop_time.usecH;
    stop_time.usecL = self->stop_time.usecL;
    is_not_yet_expired = timercmp (&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

int osal_usleep (uint32 usec){
    Delay_us(usec);
}

/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (SysTick_Config(SystemCoreClock / 1000000))  //1us
	{ 
		/* Capture error */ 
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn,0);
}

/**
  * @brief   us延时程序,10us为一个单位
  * @param  
  *		@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
  * @retval  无
  */
void Delay_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	while(TimingDelay != 0);
}

/**
  * @brief   us延时程序,10us为一个单位
  * @param  
  *		@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
  * @retval  无
  */
void Delay_ms(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	while(TimingDelay != 0);
}

/**
  * @brief  获取节拍程序
  * @param  无
  * @retval 无
  * @attention  在 SysTick 中断函数 SysTick_Handler()调用
  */
void TimingDelay_Decrement(void)
{
    sys_timer.usecL++;
	if(sys_timer.usecL>=USECS_PER_HIGH){
		++sys_timer.usecH;
	    sys_timer.usecL -= USECS_PER_HIGH;
	}
	
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}
/*********************************************END OF FILE**********************/
