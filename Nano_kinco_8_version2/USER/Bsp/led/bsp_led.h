//############################################################
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//EtherCAT��վѧϰ��
//Author��͢���������
//�Ա�����: https://shop461235811.taobao.com/
//�ҵĲ��ͣ�https://blog.csdn.net/zhandouhu/article/category/9455918
//############################################################

#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

//���Ŷ���
/*******************************************************/

//LED0 
#define LED0_PIN                  GPIO_Pin_8                 
#define LED0_GPIO_PORT            GPIOE                      
#define LED0_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED1 
#define LED1_PIN                  GPIO_Pin_9                 
#define LED1_GPIO_PORT            GPIOE                      
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED2
#define LED2_PIN                  GPIO_Pin_10                 
#define LED2_GPIO_PORT            GPIOE                      
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED3
#define LED3_PIN                  GPIO_Pin_11                 
#define LED3_GPIO_PORT            GPIOE                       
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED4 
#define LED4_PIN                  GPIO_Pin_12                 
#define LED4_GPIO_PORT            GPIOE                      
#define LED4_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED5 
#define LED5_PIN                  GPIO_Pin_13                 
#define LED5_GPIO_PORT            GPIOE                      
#define LED5_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED6
#define LED6_PIN                  GPIO_Pin_14                 
#define LED6_GPIO_PORT            GPIOE                      
#define LED6_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED7
#define LED7_PIN                  GPIO_Pin_15                 
#define LED7_GPIO_PORT            GPIOE                       
#define LED7_GPIO_CLK             RCC_AHB1Periph_GPIOE
/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  1
#define OFF 0

/* ���κ꣬��������������һ��ʹ�� */
#define LED0(a)	if (a)	\
					GPIO_SetBits(LED0_GPIO_PORT,LED0_PIN);\
					else		\
					GPIO_ResetBits(LED0_GPIO_PORT,LED0_PIN)

#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN)
					
#define LED4(a)	if (a)	\
					GPIO_SetBits(LED4_GPIO_PORT,LED4_PIN);\
					else		\
					GPIO_ResetBits(LED4_GPIO_PORT,LED4_PIN)

#define LED5(a)	if (a)	\
					GPIO_SetBits(LED5_GPIO_PORT,LED5_PIN);\
					else		\
					GPIO_ResetBits(LED5_GPIO_PORT,LED5_PIN)

#define LED6(a)	if (a)	\
					GPIO_SetBits(LED6_GPIO_PORT,LED6_PIN);\
					else		\
					GPIO_ResetBits(LED6_GPIO_PORT,LED6_PIN)

#define LED7(a)	if (a)	\
					GPIO_SetBits(LED7_GPIO_PORT,LED7_PIN);\
					else		\
					GPIO_ResetBits(LED7_GPIO_PORT,LED7_PIN)

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRRL=i;}			  //����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BSRRH=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define LED0_TOGGLE		digitalToggle(LED0_GPIO_PORT,LED0_PIN)
#define LED0_OFF		digitalHi(LED0_GPIO_PORT,LED0_PIN)
#define LED0_ON			digitalLo(LED0_GPIO_PORT,LED0_PIN)

#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON			digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF		digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON			digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE		digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF		digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON			digitalLo(LED3_GPIO_PORT,LED3_PIN)

#define LED4_TOGGLE		digitalToggle(LED4_GPIO_PORT,LED4_PIN)
#define LED4_OFF		digitalHi(LED4_GPIO_PORT,LED4_PIN)
#define LED4_ON			digitalLo(LED4_GPIO_PORT,LED4_PIN)

#define LED5_TOGGLE		digitalToggle(LED5_GPIO_PORT,LED5_PIN)
#define LED5_OFF		digitalHi(LED5_GPIO_PORT,LED5_PIN)
#define LED5_ON			digitalLo(LED5_GPIO_PORT,LED5_PIN)

#define LED6_TOGGLE		digitalToggle(LED6_GPIO_PORT,LED6_PIN)
#define LED6_OFF		digitalHi(LED6_GPIO_PORT,LED6_PIN)
#define LED6_ON			digitalLo(LED6_GPIO_PORT,LED6_PIN)

#define LED7_TOGGLE		digitalToggle(LED7_GPIO_PORT,LED7_PIN)
#define LED7_OFF		digitalHi(LED7_GPIO_PORT,LED7_PIN)
#define LED7_ON			digitalLo(LED7_GPIO_PORT,LED7_PIN)


#define LED_ALLOFF LED1_OFF;LED2_OFF;LED3_OFF;


//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
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
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 

//LED�˿ڶ���
//#define LED0 PEout(8)	// DS0
//#define LED1 PEout(9)	// DS1	 
//#define LED2 PEout(10)	// DS1	 


void LED_GPIO_Config(void);

#endif /* __LED_H */
