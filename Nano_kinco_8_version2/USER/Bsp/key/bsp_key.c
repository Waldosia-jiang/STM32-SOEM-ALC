//############################################################
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//EtherCAT从站学习板
//Author：廷华电子设计
//淘宝店铺: https://shop461235811.taobao.com/
//我的博客：https://blog.csdn.net/zhandouhu/article/category/9455918
//############################################################
  
#include "./Bsp/key/bsp_key.h" 

/// 不精确的延时
void Key_Delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 

/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*开启按键GPIO口的时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
  /*选择按键的引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  
  /*设置引脚为输入模式*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  
  /*设置引脚不上拉也不下拉*/
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	
  /*使用上面的结构体初始化按键*/
	GPIO_Init(GPIOD, &GPIO_InitStructure);   
  
//  /*选择按键的引脚*/
//	GPIO_InitStructure.GPIO_Pin = KEY2_PIN; 
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  
//  /*使用上面的结构体初始化按键*/
//	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);  
//	
//	/*选择按键的引脚*/
//	GPIO_InitStructure.GPIO_Pin = KEY3_PIN; 
//  
//  /*使用上面的结构体初始化按键*/
//	GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);  
}


/*********************************************END OF FILE**********************/
