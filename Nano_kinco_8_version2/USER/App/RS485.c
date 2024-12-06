#include "sys.h"		    
#include "RS485.h"	 
#include "delay.h"
#include "./Bsp/usart/bsp_debug_usart.h"
#include "bsp_MB_host.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//RS485���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/* ˽�б��� ------------------------------------------------------------------*/

  __IO  uint8_t  Tx_Buf[256];     // ���ͻ���,���256�ֽ�
  __IO  uint8_t  Rx_Buf[256];     // ���ջ���,���256�ֽ�
  __IO  uint8_t  Rx_flag = 0;         //�յ�һ֡���ݵı�־
  __IO  uint8_t  time_start = 0;      //MODbus��ʱ���Ƿ��ʱ�ı�־,0������ʱ��1�����ʱ
  __IO  uint8_t  timout = 0 ;         //MODbus�����ݶ���ʱ��
  __IO  uint8_t  RX_COUNT = 0;        //MODbus�˿��Ѿ��յ������ݸ���
	__IO  uint8_t  ERROR_COUNT = 0;     //MODbus�˿��Ѿ��յ������ݸ���
	__IO  uint8_t  MB_SLAVEADDR = 0x05;     // Ĭ�ϴӻ���ַ
  __IO  uint8_t  Host_Sendtime = 0;	

  __IO  uint8_t  RS485_10H_temp = 0;
  __IO  uint8_t  RS485_10H_temp1 = 0;
	
	__IO  uint16_t RS485_06H = 0x01;
        uint8_t  Link_status = 0;
 
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/*
��Ϊ������ 9600
1λ���ݵ�ʱ��Ϊ 1000000us/9600bit/s=104us
һ���ֽ�Ϊ    104us*10λ  =1040us
���� MODBUSȷ��һ������֡��ɵ�ʱ��Ϊ   1040us*3.5=3.64ms  ->10ms
*/

void UART4_IRQHandler(void)
{ 
	  u8 res = 0;	
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//���յ�����
	{	 	
	    
  		res = USART_ReceiveData(UART4);//;��ȡ���յ�������UART4->DR		
  		
		  if(Rx_flag == 1) return ;  //�����ݰ����ڴ���
		
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
		
      //�յ��ӻ�������һ֡���ݵĵ�һ�ֽ�		
      if(RX_COUNT == 1) time_start = 1;  //������ʱ
							
			}
			
  	  USART_ClearITPendingBit(UART4,USART_IT_RXNE);			
	}


//��ʼ��IO ����4
//bound:������	  
void RS485_Init(u32 bound)
{ 

  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��UART4ʱ��
	
  //����4���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUART4
	
	//UART4    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10��PC11
	
  //UART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ��� 4
//	USART_ClearFlag(UART4, USART_FLAG_TC);       //����жϱ�־

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//���������ж�

	//Usart4 NVIC ����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	  	for(t=0;t<len;t++)		//ѭ����������
	{
	  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //�ȴ����ͽ���		
    USART_SendData(UART4,buf[t]); //��������
	}	 
	while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //�ȴ����ͽ���		
	  
}

//RS485��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS485_Receive_Data()
{   
	  static uint16_t crc_check = 0;
	
	  if(Rx_Buf[0] != MB_SLAVEADDR || Rx_Buf[7] == 0 || Rx_Buf[6] == 0 || Rx_Buf[8] == 0) //�ӻ���ַ��������ݴ���
		 {
			    Rx_flag = 0;
			    time_start  = 0;   //�رն�ʱ��--ֹͣ��ʱ
			    RX_COUNT = 0;
					return ;
		 }
		 
		 if((Rx_Buf[4] & 0X01) == 0X01) //ʧ��
		 {
			    Link_status++;
		 }
		 else if((Rx_Buf[4] & 0X01) != 0X01) //����״̬
		 {
			    Link_status = 0;
		 }
		 
	  // ����ط���������һ�£���Ϊ��Ҫ���ǵ�ѹ���ģ���Ӱ�죬��RX_COUNT���ó�13
//	 crc_check = ((Rx_Buf[RX_COUNT-1]<<8) | Rx_Buf[RX_COUNT-2] );
		 crc_check = ((Rx_Buf[13-1]<<8) | Rx_Buf[13-2] );
		
//		printf("crc_check1 = %x \r \n" , crc_check);
//		printf("crc_check2 = %x \r \n" , MB_CRC16((uint8_t*) Rx_Buf , RX_COUNT-2));
	
		/* CRC У����ȷ */
//		if(crc_check == MB_CRC16((uint8_t*) Rx_Buf , RX_COUNT-2) && (Link_status >= 10 || Link_status == 0)){
		if(crc_check == MB_CRC16((uint8_t*) Rx_Buf , 13-2) && !Link_status){
//		if(crc_check == MB_CRC16((uint8_t*) Rx_Buf , RX_COUNT-2)){	  
//			  printf("success \n \r");
			  
			  Link_status = 0;
			
			  Move_X =  (Rx_Buf[7] - 127) / 150.0 * vel ;
        Move_Y =  (Rx_Buf[6] - 127) / 150.0;
        Move_Z =  (Rx_Buf[8] - 127) / 150.0;
        
			  int count = 0;
			  			  		  
			  // ��צץ
			  if((Rx_Buf[3] & 0X01) == 0X01 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_0);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_0);
        
        // ��צ��
//			  if((Rx_Buf[3] & 0X02) == 0X02 && count == 0 && (Rx_Buf[5] & 0X04) == 0X04){	
        if((Rx_Buf[3] & 0X02) == 0X02 && count == 0 && (Rx_Buf[5] & 0X40) == 0X40){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_1);
					count++; 
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_1);
				
        
        // ��צ����
			  if((Rx_Buf[3] & 0X04) == 0X04 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_2);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_2);
				       
        // ��צ����
			  if((Rx_Buf[3] & 0X08) == 0X08 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_3);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_3);
        
				
        // ƽצ��
			  if((Rx_Buf[4] & 0X04) == 0X04 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_4);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_4);
        
        // ƽצ��
			  if((Rx_Buf[4] & 0X08) == 0X08 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_5);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_5);
        
        // ����
			  if((Rx_Buf[4] & 0X10) == 0X10 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_6);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_6);
        
        // �½�
			  if((Rx_Buf[4] & 0X20) == 0X20 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_7);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_7);
        
        // ǰ��
			  if((Rx_Buf[4] & 0X40) == 0X40 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_8);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_8);
        
        // ����
			  if((Rx_Buf[4] & 0X80) == 0X80 && count == 0){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_9);
					count++;
				}else GPIO_ResetBits(GPIOE,GPIO_Pin_9);
				
				 // Һѹ���
			  if((Rx_Buf[3] & 0X10) == 0X10){				    
					GPIO_SetBits(GPIOE,GPIO_Pin_10);							
				}else {
					GPIO_ResetBits(GPIOE,GPIO_Pin_10);				
				}             				
         // ����24V��ѹ��ŷ� 				
				if(count == 1) GPIO_SetBits(GPIOE,GPIO_Pin_11);
				else GPIO_ResetBits(GPIOE,GPIO_Pin_11);				
				
        // ����ģʽ,Ĭ���е��ٶ�ģʽ�����̣�
			  if((Rx_Buf[5] & 0X01) == 0X01) vel = 1.2;				    
        // ����ģʽ
				else if((Rx_Buf[5] & 0X03) == 0X00) vel = 0.6;			 
        // ����ģʽ
			  else if((Rx_Buf[5] & 0X02) == 0X02) vel = 0.075;
				
         
        // ����ģʽ,Ĭ�ϵ���ģʽ��Һѹ��
			  if((Rx_Buf[5] & 0X10) == 0X10){				    
					  vel_yy = 0.7;
					   //�ܷ�����������ת���������
					  if((Rx_Buf[5] & 0X04) == 0X04) vel_yy = 1.2; 
					  RS485_10H_temp = 0X10;
				}			
				// ����ģʽ,Ĭ������ģʽ��Һѹ��
			  else if((Rx_Buf[5] & 0X30) == 0X00){				    
					  vel_yy = 0.3;
					  RS485_10H_temp = 0X00;
				}
        // ����ģʽ
			  else if((Rx_Buf[5] & 0X20) == 0X20){
				    vel_yy = 0.12;
					  RS485_10H_temp = 0X20;
				} 
				
				//�ж�Һѹ�Ƿ��ж���
				if(RS485_10H_temp != RS485_10H_temp1)  RS485_10H_FLAG = 5;					
				
				RS485_10H_temp1 = RS485_10H_temp;
        	  			
					  
			}else if(Link_status > 10){
				 	
          vel = 0.0;
          vel_yy = 0.0;				
					printf("error \n \r");
				
		  };

			Rx_flag = 0;
			time_start  = 0;   //�رն�ʱ��--ֹͣ��ʱ
			RX_COUNT = 0;
			
   }			
