#ifndef __EATHER_CONFIG_H
#define __EATHER_CONFIG_H

#include "osal.h"

//// 保存当前对齐状态，并将对齐方式设置为1字节对齐
//#pragma pack(push, 1)
typedef __packed struct
{   
	 uint16 ControlWord;
	 int32  TargetPos;
	 uint8  TargetMode;
	 int32  TargetVelocity;
	 uint32 TrapezoidVelocity;
}PDO_Output;
//// 恢复先前的对齐状态
//#pragma pack(pop)

//// 保存当前对齐状态，并将对齐方式设置为1字节对齐
//#pragma pack(push, 1)
typedef __packed struct 
{
   uint16 StatusWord;
   int32 CurrentPosition;
   int32 CurrentVelocity;
   uint16 ErrorCode;
   uint8 CurrentMode;
}PDO_Input;
//// 恢复先前的对齐状态
//#pragma pack(pop)

#ifdef __cplusplus
 extern "C" {
#endif


//int Servosetup(uint16 slave);
//void ecat_app(void);
//void simpletest(char *ifname);


#endif /* __EATHER_CONFIG_H */

