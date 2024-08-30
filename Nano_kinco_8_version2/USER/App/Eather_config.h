#ifndef __EATHER_CONFIG_H
#define __EATHER_CONFIG_H

#include "osal.h"

//// ���浱ǰ����״̬���������뷽ʽ����Ϊ1�ֽڶ���
//#pragma pack(push, 1)
typedef __packed struct
{   
	 uint16 ControlWord;
	 int32  TargetPos;
	 uint8  TargetMode;
	 int32  TargetVelocity;
	 uint32 TrapezoidVelocity;
}PDO_Output;
//// �ָ���ǰ�Ķ���״̬
//#pragma pack(pop)

//// ���浱ǰ����״̬���������뷽ʽ����Ϊ1�ֽڶ���
//#pragma pack(push, 1)
typedef __packed struct 
{
   uint16 StatusWord;
   int32 CurrentPosition;
   int32 CurrentVelocity;
   uint16 ErrorCode;
   uint8 CurrentMode;
}PDO_Input;
//// �ָ���ǰ�Ķ���״̬
//#pragma pack(pop)

#ifdef __cplusplus
 extern "C" {
#endif


//int Servosetup(uint16 slave);
//void ecat_app(void);
//void simpletest(char *ifname);


#endif /* __EATHER_CONFIG_H */

