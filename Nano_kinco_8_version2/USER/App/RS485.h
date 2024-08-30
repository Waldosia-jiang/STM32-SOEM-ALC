#ifndef __RS485_H
#define __RS485_H	

#include "sys.h"	 								  
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

extern  __IO  uint8_t  Tx_Buf[256];      // ���ͻ���,���256�ֽ�
extern  __IO  uint8_t  Rx_Buf[256];      // ���ջ���,���256�ֽ�
extern  __IO  uint8_t  Rx_flag;          //�յ�һ֡���ݵı�־
extern  __IO  uint8_t  time_start;       //MODbus��ʱ���Ƿ��ʱ�ı�־,0������ʱ��1�����ʱ
extern  __IO  uint8_t  timout;           //MODbus�����ݶ���ʱ��
extern  __IO  uint8_t  RX_COUNT;         //MODbus�˿��Ѿ��յ������ݸ���
extern	__IO  uint8_t  MB_SLAVEADDR;     // Ĭ�ϴӻ���ַ
extern  __IO  uint8_t  Host_Sendtime;
extern  __IO  uint8_t  RS485_10H_temp;
extern  __IO  uint8_t  RS485_10H_temp1;	

extern	__IO  uint16_t RS485_06H;


/* ��չ���� ------------------------------------------------------------------*/

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data();	

#endif	   












