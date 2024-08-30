#ifndef __RS485_H
#define __RS485_H	

#include "sys.h"	 								  
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

extern  __IO  uint8_t  Tx_Buf[256];      // 发送缓存,最大256字节
extern  __IO  uint8_t  Rx_Buf[256];      // 接收缓存,最大256字节
extern  __IO  uint8_t  Rx_flag;          //收到一帧数据的标志
extern  __IO  uint8_t  time_start;       //MODbus定时器是否计时的标志,0代表不计时，1代表计时
extern  __IO  uint8_t  timout;           //MODbus的数据断续时间
extern  __IO  uint8_t  RX_COUNT;         //MODbus端口已经收到的数据个数
extern	__IO  uint8_t  MB_SLAVEADDR;     // 默认从机地址
extern  __IO  uint8_t  Host_Sendtime;
extern  __IO  uint8_t  RS485_10H_temp;
extern  __IO  uint8_t  RS485_10H_temp1;	

extern	__IO  uint16_t RS485_06H;


/* 扩展变量 ------------------------------------------------------------------*/

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data();	

#endif	   












