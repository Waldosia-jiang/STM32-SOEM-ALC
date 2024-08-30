//############################################################
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//EtherCAT从站学习板
//Author：廷华电子设计
//淘宝店铺: https://shop461235811.taobao.com/
//我的博客：https://blog.csdn.net/zhandouhu/article/category/9455918
//############################################################

#ifndef __KEY_H
#define	__KEY_H

#include "stm32f4xx.h"

////引脚定义
///*******************************************************/
//#define KEY1_PIN                  GPIO_Pin_0                 
//#define KEY1_GPIO_PORT            GPIOD                      
//#define KEY1_GPIO_CLK             RCC_AHB1Periph_GPIOD

//#define KEY2_PIN                  GPIO_Pin_1                 
//#define KEY2_GPIO_PORT            GPIOD                      
//#define KEY2_GPIO_CLK             RCC_AHB1Periph_GPIOD

//#define KEY3_PIN                  GPIO_Pin_2                
//#define KEY3_GPIO_PORT            GPIOD                 
//#define KEY3_GPIO_CLK             RCC_AHB1Periph_GPIOD
///*******************************************************/

// /** 按键按下标置宏
//	* 按键按下为高电平，设置 KEY_ON=1， KEY_OFF=0
//	* 若按键按下为低电平，把宏设置成KEY_ON=0 ，KEY_OFF=1 即可
//	*/
//#define KEY_ON	1
//#define KEY_OFF	0


//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_Addr(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_Addr(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

#define SWITCH_1            PDin(0)				// PORTDbits.RD7 /**< \brief Access to switch 1 input*/
#define SWITCH_2            PDin(1)				// PORTDbits.RD6 /**< \brief Access to switch 2 input*/
#define SWITCH_3            PDin(2)				// PORTDbits.RD5 /**< \brief Access to switch 3 input*/
#define SWITCH_4            PDin(3)				// PORTDbits.RD4 /**< \brief Access to switch 4 input*/
#define SWITCH_5            PDin(4)				// PORTDbits.RD3 /**< \brief Access to switch 5 input*/
#define SWITCH_6            PDin(5)				// PORTDbits.RD2 /**< \brief Access to switch 6 input*/
#define SWITCH_7            PDin(6)				// PORTDbits.RD1 /**< \brief Access to switch 7 input*/
#define SWITCH_8            PDin(7)				// PORTDbits.RD0 /**< \brief Access to switch 8 input*/

void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin);

#endif /* __LED_H */

