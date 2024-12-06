#include "sys.h"		    
#include "RS485.h"	 
#include "delay.h"
#include "./Bsp/usart/bsp_debug_usart.h"
#include "bsp_MB_host.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//RS485驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/* 私有变量 ------------------------------------------------------------------*/

  __IO  uint8_t  Tx_Buf[256];     // 发送缓存,最大256字节
  __IO  uint8_t  Rx_Buf[256];     // 接收缓存,最大256字节
  __IO  uint8_t  Rx_flag = 0;         //收到一帧数据的标志
  __IO  uint8_t  time_start = 0;      //MODbus定时器是否计时的标志,0代表不计时，1代表计时
  __IO  uint8_t  timout = 0 ;         //MODbus的数据断续时间
  __IO  uint8_t  RX_COUNT = 0;        //MODbus端口已经收到的数据个数
	__IO  uint8_t  ERROR_COUNT = 0;     //MODbus端口已经收到的数据个数
	__IO  uint8_t  MB_SLAVEADDR = 0x05;     // 默认从机地址
  __IO  uint8_t  Host_Sendtime = 0;	

  __IO  uint8_t  RS485_10H_temp = 0;
  __IO  uint8_t  RS485_10H_temp1 = 0;
	
	__IO  uint16_t RS485_06H = 0x01;
        uint8_t  Link_status = 0;
 
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/*
因为波特率 9600
1位数据的时间为 1000000us/9600bit/s=104us
一个字节为    104us*10位  =1040us
所以 MODBUS确定一个数据帧完成的时间为   1040us*3.5=3.64ms  ->10ms
*/

void UART4_IRQHandler(void)
{ 
	  u8 res = 0;	
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//接收到数据
	{	 	
	    
  		res = USART_ReceiveData(UART4);//;读取接收到的数据UART4->DR		
  		
		  if(Rx_flag == 1) return ;  //有数据包正在处理
		
//      if(RX_COUNT == 0 && res != 0x05) {
//	    
//			   RX_COUNT = 0;
//			   timout = 0;
//			   Rx_flag = 0;			
//			}
//		  
//			for(int i = 0; i < 13 ; i++)	 printf("%d %x \r \n ", i , Rx_Buf[i] );
		
		  Rx_Buf[RX_COUNT++] = res;
		  timout =  0;
		
      //收到从机发来的一帧数据的第一字节		
      if(RX_COUNT == 1) time_start = 1;  //启动定时
							
			}
			
  	  USART_ClearITPendingBit(UART4,USART_IT_RXNE);			
	}


//初始化IO 串口4
//bound:波特率	  
void RS485_Init(u32 bound)
{ 

  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟
	
  //串口4引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4
	
	//UART4    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10与GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11
	
  //UART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(UART4, ENABLE);  //使能串口 4
//	USART_ClearFlag(UART4, USART_FLAG_TC);       //清除中断标志

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启接受中断

	//Usart4 NVIC 配置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	  	for(t=0;t<len;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //等待发送结束		
    USART_SendData(UART4,buf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //等待发送结束		
	  
}

//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data()
{   
	  static uint16_t crc_check = 0;
	
	  if(Rx_Buf[0] != MB_SLAVEADDR || Rx_Buf[7] == 0 || Rx_Buf[6] == 0 || Rx_Buf[8] == 0) //从机地址错误或数据错误
		 {
			    Rx_flag = 0;
			    time_start  = 0;   //关闭定时器--停止定时
			    RX_COUNT = 0;
					return ;
		 }
		 
		 if((Rx_Buf[4] & 0X01) == 0X01) //失联
		 {
			    Link_status++;
		 }
		 else if((Rx_Buf[4] & 0X01) != 0X01) //连接状态
		 {
			    Link_status = 0;
		 }
		 
	  // 这个地方单独处理一下，因为需要考虑电压输出模块的影响，将RX_COUNT设置成13
//	 crc_check = ((Rx_Buf[RX_COUNT-1]<<8) | Rx_Buf[RX_COUNT-2] );
		 crc_check = ((Rx_Buf[13-1]<<8) | Rx_Buf[13-2] );
		
//		printf("crc_check1 = %x \r \n" , crc_check);
//		printf("crc_check2 = %x \r \n" , MB_CRC16((uint8_t*) Rx_Buf , RX_COUNT-2));
	
		/* CRC 校验正确 */
//		if(crc_check == MB_CRC16((uint8_t*) Rx_Buf , RX_COUNT-2) && (Link_status >= 10 || Link_status == 0)){
		if(crc_check == MB_CRC16((uint8_t*) Rx_Buf , 13-2) && !Link_status){
//		if(crc_check == MB_CRC16((uint8_t*) Rx_Buf , RX_COUNT-2)){	  
//			  printf("success \n \r");
			  
			  Link_status = 0;
			
			  Move_X =  (Rx_Buf[7] - 127) / 150.0 * vel ;
        Move_Y =  (Rx_Buf[6] - 127) / 150.0;
        Move_Z =  (Rx_Buf[8] - 127) / 150.0;
        
			  int count = 0;
			  			  		  
			  // 主爪抓
			  if((Rx_Buf[3] & 0X01) == 0X01 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_0);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_0);
        
        // 主爪松
//			  if((Rx_Buf[3] & 0X02) == 0X02 && count == 0 && (Rx_Buf[5] & 0X04) == 0X04){	
        if((Rx_Buf[3] & 0X02) == 0X02 && count == 0 && (Rx_Buf[5] & 0X40) == 0X40){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_1);
					count++; 
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_1);
				
        
        // 副爪正向
			  if((Rx_Buf[3] & 0X04) == 0X04 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_2);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_2);
				       
        // 副爪反向
			  if((Rx_Buf[3] & 0X08) == 0X08 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_3);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_3);
        
				
        // 平爪收
			  if((Rx_Buf[4] & 0X04) == 0X04 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_4);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_4);
        
        // 平爪放
			  if((Rx_Buf[4] & 0X08) == 0X08 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_5);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_5);
        
        // 上升
			  if((Rx_Buf[4] & 0X10) == 0X10 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_6);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_6);
        
        // 下降
			  if((Rx_Buf[4] & 0X20) == 0X20 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_7);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_7);
        
        // 前移
			  if((Rx_Buf[4] & 0X40) == 0X40 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_8);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_8);
        
        // 后移
			  if((Rx_Buf[4] & 0X80) == 0X80 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_9);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_9);
				
				 // 液压电机
			  if((Rx_Buf[3] & 0X10) == 0X10){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_10);							
				}else {
					GPIO_ResetBits(GPIOE,GPIO_Pin_10);				
				}             				
         // 控制24V建压电磁阀 				
				if(count == 1) GPIO_SetBits(GPIOE,GPIO_Pin_11);
				else GPIO_ResetBits(GPIOE,GPIO_Pin_11);				
				
        // 高速模式,默认中等速度模式（底盘）
			  if((Rx_Buf[5] & 0X01) == 0X01) vel = 1.2;				    
        // 正常模式
				else if((Rx_Buf[5] & 0X03) == 0X00) vel = 0.6;			 
        // 低速模式
			  else if((Rx_Buf[5] & 0X02) == 0X02) vel = 0.075;
				
         
        // 高速模式,默认低速模式（液压）
			  if((Rx_Buf[5] & 0X10) == 0X10){				    
					  vel_yy = 0.7;
					   //总阀开单独控制转不动的情况
					  if((Rx_Buf[5] & 0X04) == 0X04) vel_yy = 1.2; 
					  RS485_10H_temp = 0X10;
				}			
				// 正常模式,默认正常模式（液压）
			  else if((Rx_Buf[5] & 0X30) == 0X00){				    
					  vel_yy = 0.3;
					  RS485_10H_temp = 0X00;
				}
        // 低速模式
			  else if((Rx_Buf[5] & 0X20) == 0X20){
				    vel_yy = 0.12;
					  RS485_10H_temp = 0X20;
				} 
				
				//判断液压是否有动作
				if(RS485_10H_temp != RS485_10H_temp1)  RS485_10H_FLAG = 5;					
				
				RS485_10H_temp1 = RS485_10H_temp;
        	  			
					  
			}else if(Link_status > 10){
				 	
          vel = 0.0;
          vel_yy = 0.0;				
					printf("error \n \r");
				
		  };

			Rx_flag = 0;
			time_start  = 0;   //关闭定时器--停止定时
			RX_COUNT = 0;
			
   }			
