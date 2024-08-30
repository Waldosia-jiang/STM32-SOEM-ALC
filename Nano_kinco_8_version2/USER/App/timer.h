#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h" 
#include "misc.h" 
#include "stm32f4xx_rcc.h"

#include "osal.h"

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"


void TIM8_Cap_Init(uint16 arr, uint16 psc);
void Remote_Control(void);
int target_limit_int(int insert,int low,int high);
extern int L_Remoter_Ch1,L_Remoter_Ch2,L_Remoter_Ch3,L_Remoter_Ch4;
extern int Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;

extern float RC_Velocity;
extern volatile float Move_X,Move_Y,Move_Z; 
extern volatile float vel;
extern volatile float vel_yy;
extern volatile uint8_t  RS485_10H_FLAG;   
extern volatile uint8_t  DATA_10H[64];

#endif
