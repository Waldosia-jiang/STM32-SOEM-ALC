//############################################################
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//EtherCAT��վѧϰ��
//Author��͢���������
//�Ա�����: https://shop461235811.taobao.com/
//�ҵĲ��ͣ�https://blog.csdn.net/zhandouhu/article/category/9455918
//############################################################
  
#include "./Bsp/key/bsp_key.h" 

/// ����ȷ����ʱ
void Key_Delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 

/**
  * @brief  ���ð����õ���I/O��
  * @param  ��
  * @retval ��
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*��������GPIO�ڵ�ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
  /*ѡ�񰴼�������*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  
  /*��������Ϊ����ģʽ*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  
  /*�������Ų�����Ҳ������*/
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	
  /*ʹ������Ľṹ���ʼ������*/
	GPIO_Init(GPIOD, &GPIO_InitStructure);   
  
//  /*ѡ�񰴼�������*/
//	GPIO_InitStructure.GPIO_Pin = KEY2_PIN; 
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  
//  /*ʹ������Ľṹ���ʼ������*/
//	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);  
//	
//	/*ѡ�񰴼�������*/
//	GPIO_InitStructure.GPIO_Pin = KEY3_PIN; 
//  
//  /*ʹ������Ľṹ���ʼ������*/
//	GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);  
}


/*********************************************END OF FILE**********************/
