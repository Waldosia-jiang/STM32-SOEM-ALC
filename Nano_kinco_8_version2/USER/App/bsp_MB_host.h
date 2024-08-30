/**
  ******************************************************************************
  * �ļ�����: bsp_MB_host.h 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2018-09-01
  * ��    ��: MODBUS-API
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-H7Multiʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
#ifndef __BSP_MB_HOST_H__
#define __BSP_MB_HOST_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx.h"                  // Device header

/* ���Ͷ��� ------------------------------------------------------------------*/

/* �궨�� --------------------------------------------------------------------*/
#define MB_REG_ADDR         0x0001 //�Ĵ�����ַ����ɢ����Ȧ��
#define HoldingReg          0x0010 //���ּĴ���
#define InputRegReg         0x0020 //����Ĵ���

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
uint16_t MB_CRC16(uint8_t *pushMsg,uint8_t usDataLen);
void MB_ReadCoil_01H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_WriteCoil_05H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadInput_02H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadHoldingReg_03H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadInputReg_04H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_WriteHoldingReg_06H(uint8_t _addr, uint16_t _reg, uint16_t _data);
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf);

#endif /* __BSP_MB_HOST_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
