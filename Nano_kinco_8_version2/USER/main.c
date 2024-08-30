/*----------------------------------------------------------------------------*/
//版权信息：    淘宝店铺：班特科技
//文件名：      
//当前版本：    1.0
//单片机型号：  STM32
//开发环境：    Keil
//晶振频率：    8M
//作者：        paul
//功能：        
//修订记录:    
//2015-09-28----创建
/*----------------------------------------------------------------------------*/
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "./Bsp/led/bsp_led.h" 
#include "./Bsp/usart/bsp_debug_usart.h"
#include "./Bsp/systick/bsp_SysTick.h"
#include "./Bsp/key/bsp_key.h"
#include "netconf.h"
#include "LAN8742A.h"
#include <math.h>

#include "osal.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#include "Eather_config.h"
#include "timer.h"
#include "RS485.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* 私有类型定义 --------------------------------------------------------------*/


/* 私有宏定义 ----------------------------------------------------------------*/
#define MSG_ERR_FLAG  0xFFFF         // 接收错误 字符间超时
#define MSG_IDLE      0x0000         // 空闲状态
#define MSG_RXING     0x0001         // 正在接收数据
#define MSG_COM       0x0002         // 接收完成
#define MSG_INC       0x8000         // 数据帧不完整(两字符间的空闲间隔大于1.5个字符时间)
#define TIME_OVERRUN  200            // 定义超时时间 ms

/* 私有变量 ------------------------------------------------------------------*/
__IO uint16_t Rx_MSG = MSG_IDLE;     // 接收报文状态
__IO uint8_t  rx_flag=0,check_flag=0;


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
extern __IO uint8_t EthLinkStatus;

/* Private function prototypes -----------------------------------------------*/
static void TIM3_Config(uint16_t period , uint16_t prescaler);
/* Private functions ---------------------------------------------------------*/
//#define TRUE                1
//#define FALSE               0

extern int ec_slavecount;

//char IOmap[4096];

#define EC_TIMEOUTMON 500
#define SYNC0TIME 1000

#define D0 0.42
#define L0 0.49

#define M_PI 3.14159265358979323846
#define Min_Turn_Radius  1.5      // 最小转弯半径2m
#define Wheel_Radius     0.105  // 半径

int CAR_MODE = 1; //小车的运动模式，默认是前后轮反向偏转运动

uint64 app_time_base = 0;
__IO uint32_t pdoTimeFlag = 0;
int dorun = 0;
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */

//motor control 
uint16 cur_status1;
uint16 cur_status2;

uint16 cur_status3;
uint16 cur_status4;

uint16 cur_status5;
uint16 cur_status6;

uint16 cur_status7;
uint16 cur_status8;

PDO_Output *outputs1;
PDO_Input  *inputs1;
PDO_Output *outputs2;
PDO_Input  *inputs2;

PDO_Output *outputs3;
PDO_Input *inputs3;
PDO_Output *outputs4;
PDO_Input *inputs4;

PDO_Output *outputs5;
PDO_Input *inputs5;
PDO_Output *outputs6;
PDO_Input *inputs6;

PDO_Output *outputs7;
PDO_Input *inputs7;
PDO_Output *outputs8;
PDO_Input *inputs8;

int DCdiff;

int32 temp2 =0;
int32 temp4 =0;
int32 temp6 =0;
int32 temp8 =0;

int32 temp1 =0;
int32 temp3 =0;
int32 temp5 =0;
int32 temp7 =0;

uint8_t startup_step = 0;

uint8_t servo1_on_setp = 0;
uint8_t servo2_on_setp = 0;
uint8_t servo3_on_setp = 0;
uint8_t servo4_on_setp = 0;
uint8_t servo5_on_setp = 0;
uint8_t servo6_on_setp = 0;
uint8_t servo7_on_setp = 0;
uint8_t servo8_on_setp = 0;

uint8_t servo1_on = 0 ;
uint8_t servo2_on = 0;
uint8_t servo3_on = 0;
uint8_t servo4_on = 0;
uint8_t servo5_on = 0;
uint8_t servo6_on = 0;
uint8_t servo7_on = 0;
uint8_t servo8_on = 0;

int oloop, iloop;

char IOmap[1024];
boolean needlf;
boolean inOP;

int32 cur_pos[9] = {0};

int expectedWKC;

volatile int wkc;

uint8 currentgroup = 0;

uint64_t sss = 1000000000000000000L;
uint64_t dc_ref = 0;
//long long app_time_base = 0;

uint64_t app_time = 0;
uint32 app_time_last = 0;

int32 pll_limit = SYNC0TIME / 1000;

float  pll_p = 0.005;
float  pll_i = 0.01;
double pll_isum = 0.0;
double periodfp = (double) SYNC0TIME * 0.000000001;


uint64 ref_time_base = 0;
uint64 sync_start_time = 0;
int64  app_time_offset = 0;

uint64_t ref, now;
int32    dc_time_valid = 1;
uint32   dc_time = 0;
uint32   dc_time_last = 0;
int32    pll_err = 0;
int32    pll_out = 0;

extern int8_t ethernetif_init(void);

uint8 flag_time = 0;

uint16 csp_pos_delay;
int    cmdpos_raw;

int Servosetup(uint16 slave)
{	
	int retval;
	uint16 u16val;
	uint8  u8val;
	uint32 u32val;
	retval = 0;

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1601;	
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	
	  u8val = 0;
    retval += ec_SDOwrite(slave, 0x1601, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60400010; //	ControlWord
    retval += ec_SDOwrite(slave, 0x1601, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x607A0020; //	TargetPos
    retval += ec_SDOwrite(slave, 0x1601, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60600008; //	TargetMode
    retval += ec_SDOwrite(slave, 0x1601, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60FF0020; //	TargetVelocity
    retval += ec_SDOwrite(slave, 0x1601, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60810020; //	TrapezoidVelocity
    retval += ec_SDOwrite(slave, 0x1601, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u8val = 5;
    retval += ec_SDOwrite(slave, 0x1601, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1a00;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  
		u8val = 0;
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	  u32val = 0x60410010;	// StatusWord
    retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60640020;  // CurrentPosition
    retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x606C0020;	// CurrentVelocity
    retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x603F0010;	// ErrorCode
    retval += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
		u32val = 0x60610008;	// CurrentMode
    retval += ec_SDOwrite(slave, 0x1A00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);		
		u8val = 5;
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
		
    return 1;
}

void ecat_app(void){

	static int16_t i = 0;
	static uint8_t flag=0;
	
	uint8_t wkc;
	
			wkc = ec_receive_processdata(EC_TIMEOUTRET);
			//printf("%d ",wkc);
	    
	    if((inputs1->StatusWord & 0x0008) != 0x0008)
			cur_status1 = inputs1->StatusWord & 0x00FF;//slave1
			if((inputs2->StatusWord & 0x0008) != 0x0008)
			cur_status2 = inputs2->StatusWord & 0x00FF;//slave2
			if((inputs3->StatusWord & 0x0008) != 0x0008)
	    cur_status3 = inputs3->StatusWord & 0x00FF;//slave3
			if((inputs4->StatusWord & 0x0008) != 0x0008)
			cur_status4 = inputs4->StatusWord & 0x00FF;//slave4
			if((inputs5->StatusWord & 0x0008) != 0x0008)
	    cur_status5 = inputs5->StatusWord & 0x00FF;//slave5
			if((inputs6->StatusWord & 0x0008) != 0x0008)
			cur_status6 = inputs6->StatusWord & 0x00FF;//slave6
			if((inputs7->StatusWord & 0x0008) != 0x0008)
	    cur_status7 = inputs7->StatusWord & 0x00FF;//slave7
			if((inputs8->StatusWord & 0x0008) != 0x0008)
			cur_status8 = inputs8->StatusWord & 0x00FF;//slave8
			
	
			double  R = 65535.0;
			
	    double  V_1 = 0,V_2 = 0,V_3 = 0,V_4 = 0;
			double  angle_1 = 0,angle_2 = 0,angle_3 = 0,angle_4 = 0;
			
  		double  V =   - Move_X ; //线速度
			
//			printf("vel = %f \n \r",  vel);
//			printf("vel_yy = %f \n \r",  vel_yy);
			
			double  W =   Move_Y * 1.0; //角速度
			double  Z =   Move_Z;       //原地转圈或者横移速度
			
			int reverse = -1;
			
			//原地旋转
			if(fabs(Z) >= 0.3 && Z >= 0.3) {
				
//             printf("circle \n \r");
				       
				       if(fabs(V) >= 0.5)  V = (fabs(V) / V) * 0.5;
				   
							 V_1 =  V;
							 V_2 =  V;
				       V_3 =  V;
							 V_4 =  V;

							 angle_1 =   M_PI / 2 - atan(D0/L0);
							 angle_2 = -(M_PI / 2 - atan(D0/L0));
				       angle_3 =   M_PI / 2 - atan(D0/L0);
							 angle_4 = -(M_PI / 2 - atan(D0/L0));
			}
			
			//左右横移
			if(fabs(Z) >= 0.3 && Z <= -0.3) { 
               
//				     printf("left_right \n \r");
							 V_1 =    V;
							 V_2 =   -V;
				       V_3 =   -V;
							 V_4 =    V;
						   
				       int reverse = 1;

							 angle_1 =     1 / 4.0 * (2 * M_PI);
							 angle_2 =    -1 / 4.0 * (2 * M_PI);
				       angle_3 =     1 / 4.0 * (2 * M_PI);
							 angle_4 =    -1 / 4.0 * (2 * M_PI);
			}			
			//前后反向偏转运动
			if(fabs(V) >= 0.5 && fabs(W) >= 0.5 && fabs(Z) < 0.15){	
				     
//				   printf("forword_back \n \r");
						 R =  W / fabs(W) * fabs(V / W);
						
						 if(fabs(R) < Min_Turn_Radius){
							 
						 if(W > 0){
							 
									W =  -fabs(V / Min_Turn_Radius);
									R =   fabs(V / W);
						 } 
						 else{
							 
									W =  fabs(V / Min_Turn_Radius);
									R =  fabs(V / W);
						 }      	 
					 }else{
					 
					 if(W > 0){
						 V_1 =  - V * sqrt((R + D0)*(R + D0) + L0 * L0) / fabs(R);
					   V_2 =  - V * sqrt((R + D0)*(R + D0) + L0 * L0) / fabs(R);
					   V_3 =    V * sqrt((R - D0)*(R - D0) + L0 * L0) / fabs(R);
					   V_4 =    V * sqrt((R - D0)*(R - D0) + L0 * L0) / fabs(R);
						 
						 angle_1 =  atan((L0/(R+D0)));
					   angle_2 = -atan((L0/(R+D0)));
					   angle_3 = -atan((L0/(R-D0)));
					   angle_4 =  atan((L0/(R-D0)));
						 
					 }
					 else if(W < 0){
						 V_1 =  - V * sqrt((R - D0)*(R - D0) + L0 * L0) / fabs(R);
					   V_2 =  - V * sqrt((R - D0)*(R - D0) + L0 * L0) / fabs(R);
					   V_3 =    V * sqrt((R + D0)*(R + D0) + L0 * L0) / fabs(R);
					   V_4 =    V * sqrt((R + D0)*(R + D0) + L0 * L0) / fabs(R);
						 
						 angle_1 =  -atan((L0/(R-D0)));
					   angle_2 =   atan((L0/(R-D0)));
					   angle_3 =   atan((L0/(R+D0)));
					   angle_4 =  -atan((L0/(R+D0)));					 
					 }					 
				 }
						 	
      //直线运动					 
			}else if(fabs(V) >= 0.015 && fabs(W) <= 0.15 && fabs(Z) <= 0.15){
				
					 V_1 = -V ;
					 V_2 = -V ;
				   V_3 =  V ;
					 V_4 =  V ;
					
					 angle_1 = 0;
					 angle_2 = 0;	
           angle_3 = 0;
					 angle_4 = 0;					
			}		
//			printf(" angle_1_4=%f , angle_2_3=%f , ", angle_1_4 , angle_2_3);
			
					switch(servo1_on_setp)
								{
										case 1:
											
											if(cur_status1 == 0x0031)
													servo1_on_setp=2;
											outputs1->ControlWord = 0x07;//0x6040
												break;
										case 2:
												           
											if(cur_status1==0x0033)
											    servo1_on_setp=3;
											outputs1->ControlWord = 0x0F;  
												break;
										case 3:              
											if(cur_status1==0x0037)
												 servo1_on = 1;										
												break;            
										default :
												servo1_on_setp=1;
												outputs1->ControlWord = 0x06;//0x6040
										break;
								}
					switch(servo2_on_setp)
					{
							case 1:
								outputs2->ControlWord = 0x07;//0x6040
								if(cur_status2 == 0x0031)
											 servo2_on_setp=2;
									break;
							case 2:
									outputs2->ControlWord = 0x0F;             
											if(cur_status2==0x0033)
											 servo2_on_setp=3;
									break;
							case 3:              
											if(cur_status2==0x0037)
											 servo2_on = 1;											
									break;            
							default :
									servo2_on_setp=1;
									outputs2->ControlWord = 0x06;//0x6040
							break;
					}
					switch(servo3_on_setp)
					{
							case 1:
								outputs3->ControlWord = 0x07;//0x6040
								if(cur_status3 == 0x0031)
											 servo3_on_setp=2;
									break;
							case 2:
									outputs3->ControlWord = 0x0F;             
											if(cur_status3==0x0033)
											 servo3_on_setp=3;
									break;
							case 3:              
											if(cur_status3==0x0037)
											 servo3_on = 1;											
									break;            
							default :
									servo3_on_setp=1;
									outputs3->ControlWord = 0x06;//0x6040
							break;
					}
					switch(servo4_on_setp)
					{
							case 1:
								outputs4->ControlWord = 0x07;//0x6040
								if(cur_status4 == 0x0031)
											 servo4_on_setp=2;
									break;
							case 2:
									outputs4->ControlWord = 0x0F;             
											if(cur_status4==0x0033)
											 servo4_on_setp=3;
									break;
							case 3:              
											if(cur_status4==0x0037)
											 servo4_on = 1;											
									break;            
							default :
									servo4_on_setp=1;
									outputs4->ControlWord = 0x06;//0x6040
							break;
					}
					switch(servo5_on_setp)
					{
							case 1:
								outputs5->ControlWord = 0x07;//0x6040
								if(cur_status5 == 0x0031)
											 servo5_on_setp=2;
									break;
							case 2:
									outputs5->ControlWord = 0x0F;             
											if(cur_status5==0x0033)
											 servo5_on_setp=3;											
									break;
							case 3:              
											if(cur_status5==0x0037)
											 servo5_on = 1;											
									break;            
							default :
									servo5_on_setp=1;
									outputs5->ControlWord = 0x06;//0x6040
							break;
					}
					switch(servo6_on_setp)
					{
							case 1:
								outputs6->ControlWord = 0x07;//0x6040
								if(cur_status6 == 0x0031)
											 servo6_on_setp=2;
									break;
							case 2:
									outputs6->ControlWord = 0x0F;             
											if(cur_status6==0x0033)
											 servo6_on_setp=3;
									break;
							case 3:              
											if(cur_status6==0x0037)
											 servo6_on = 1;											
									break;            
							default :
									servo6_on_setp=1;
									outputs6->ControlWord = 0x06;//0x6040
							break;
					}
					switch(servo7_on_setp)
					{
							case 1:
								outputs7->ControlWord = 0x07;//0x7040
								if(cur_status7 == 0x0031)
											 servo7_on_setp=2;
									break;
							case 2:
									outputs7->ControlWord = 0x0F;             
											if(cur_status7==0x0033)
											 servo7_on_setp=3;
									break;
							case 3:              
											if(cur_status7==0x0037)
											 servo7_on = 1;											
									break;            
							default :
									servo7_on_setp=1;
									outputs7->ControlWord = 0x06;//0x6040
							break;
					}
					switch(servo8_on_setp)
					{
							case 1:
								outputs8->ControlWord = 0x07;//0x7040
								if(cur_status8 == 0x0031)
											 servo8_on_setp=2;
									break;
							case 2:
									outputs8->ControlWord = 0x0F;             
											if(cur_status8==0x0033)
											 servo8_on_setp=3;
									break;
							case 3:              
											if(cur_status8==0x0037)
											 servo8_on = 1;											
									break;            
							default :
									servo8_on_setp=1;
									outputs8->ControlWord = 0x06;//0x6040
							break;
					}
			
          if(servo1_on == 1 && servo2_on == 1  && servo3_on == 1 && servo4_on == 1 &&
						 servo5_on == 1 && servo6_on == 1  && servo8_on == 1 && servo8_on == 1
					){
						
					  //点亮绿色警示灯
			     GPIO_SetBits(GPIOE,GPIO_Pin_12);
						
					 RS485_06H = 0x03;

           outputs1->TargetMode = 0x3;
				   outputs1->ControlWord = 0xf;
  				 outputs1->TargetVelocity = V_1 * 12145423; //
				   outputs1->TrapezoidVelocity = 0.1 * 2731000; // 1000rpm/min						
				
					 outputs2->TargetMode = 0x1;
				   outputs2->ControlWord = 0x103f;
				   outputs2->TargetVelocity = 0.3 * 1.6 * 32772000; // 3000rpm/min
				   outputs2->TrapezoidVelocity = 1.02 * 1.2 * 32772000; // 2000rpm/min
				   
				   outputs3->TargetMode = 0x3;
				   outputs3->ControlWord = 0xf;
  				 outputs3->TargetVelocity = V_2 * 12145423; //
				   outputs3->TrapezoidVelocity = 0.1 * 2731000; // 1000rpm/min
				
					 outputs4->TargetMode = 0x1;
				   outputs4->ControlWord = 0x103f;
				   outputs4->TargetVelocity = 0.3 * 1.6 * 32772000; // 3000rpm/min
				   outputs4->TrapezoidVelocity = 1.02 * 1.2 * 32772000; // 2000rpm/min
				
				   outputs5->TargetMode = 0x3;
				   outputs5->ControlWord = 0xf;
				   outputs5->TargetVelocity = V_3 * 12145423; // 					 
				   outputs5->TrapezoidVelocity = 0.1 * 2731000; // 1000rpm/min
				
					 outputs6->TargetMode = 0x1;
				   outputs6->ControlWord = 0x103f;
				   outputs6->TargetVelocity = 0.3 * 1.6 * 32772000; // 3000rpm/min
				   outputs6->TrapezoidVelocity = 1.02 * 1.2 * 32772000; // 2000rpm/min
				   
				   outputs7->TargetMode = 0x3;
				   outputs7->ControlWord = 0xf;
				   outputs7->TargetVelocity = V_4 * 12145423; // 3000rpm/min
				   outputs7->TrapezoidVelocity = 0.1 * 2731000; // 1000rpm/min
				
					 outputs8->TargetMode = 0x1;
				   outputs8->ControlWord = 0x103f;
				   outputs8->TargetVelocity = 0.3 * 1.6 * 32772000; // 3000rpm/min
				   outputs8->TrapezoidVelocity = 1.02 * 1.2 * 32772000; // 2000rpm/min
									
  			  for(int i = 1 ;i <= 8;i++){	
						
						       if(i == 1) cur_pos[1] = inputs1->CurrentPosition;
						  else if(i == 2) cur_pos[2] = inputs2->CurrentPosition;
						  else if(i == 3) cur_pos[3] = inputs3->CurrentPosition;
						  else if(i == 4) cur_pos[4] = inputs4->CurrentPosition;
						  else if(i == 5) cur_pos[5] = inputs5->CurrentPosition;
						  else if(i == 6) cur_pos[6] = inputs6->CurrentPosition;
						  else if(i == 7) cur_pos[7] = inputs7->CurrentPosition;
						  else if(i == 8) cur_pos[8] = inputs8->CurrentPosition;
						
					}							  
					
					 temp1 = angle_1 / (2 * M_PI) * 65536 * 275;
			
					 if(abs(cur_pos[2] - temp1) > 40000 ) {									 
						 outputs2->TargetPos =   angle_1 / (2 * M_PI) * 65536 * 275;						 
						 temp2 = outputs2->TargetPos;                 							 
					 }else{							
						outputs2->TargetPos = temp2;															 
					 }
					 
					 temp3 =  angle_2 / (2 * M_PI) * 65536 * 275;

					 if(abs(cur_pos[4] - temp3) > 40000 ) {
						 outputs4->TargetPos =  angle_2 / (2 * M_PI) * 65536 * 275;	   
						 temp4 = outputs4->TargetPos;		 
					 }else{														    
						outputs4->TargetPos = temp4;										 
					 }
					 
					 temp5 = angle_3 / (2 * M_PI) * 65536 * 275;

					 if(abs(cur_pos[6] - temp5) > 40000 ) {
						 outputs6->TargetPos =  angle_3 / (2 * M_PI) * 65536 * 275;	   
						 temp6 = outputs6->TargetPos;		 
					 }else{														    
						outputs6->TargetPos = temp6;										 
					 }
					 
					 temp7 = angle_4 / (2 * M_PI) * 65536 * 275;

					 if(abs(cur_pos[8] - temp7) > 40000 ) {
						 outputs8->TargetPos = angle_4 / (2 * M_PI) * 65536 * 275;   
						 temp8 = outputs8->TargetPos;		 
					 }else{														    
						outputs8->TargetPos = temp8;										 
					 }

					if((inputs1->StatusWord & 0x0008) == 0x0008)
					{
						outputs1->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);	
					}
					
					if((inputs2->StatusWord & 0x0008) == 0x0008)						
					{
						outputs2->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);						
					}
					
					if((inputs3->StatusWord & 0x0008) == 0x0008)						
					{
						outputs3->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);					
					}
					
					if((inputs4->StatusWord & 0x0008) == 0x0008)
					{
						outputs4->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);						
					}
					
					if((inputs5->StatusWord & 0x0008) == 0x0008)
					{
						outputs5->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);					
					}
					
					if((inputs6->StatusWord & 0x0008) == 0x0008)
					{
						outputs6->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);						
					}
					
					if((inputs7->StatusWord & 0x0008) == 0x0008)
					{						
						outputs7->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);					
					}
					
					if((inputs8->StatusWord & 0x0008) == 0x0008)
					{
						outputs8->ControlWord = 0x86; //
					  GPIO_ResetBits(GPIOE,GPIO_Pin_12);						
					}
	  }else{
			   //关闭绿色警示灯
	       GPIO_ResetBits(GPIOE,GPIO_Pin_12);		
	  }
	
			ec_send_processdata();	
		  ec_pdo_outframe();	

}

void simpletest(char *ifname)
{
	int i, j, wkc_count, chk, slc;
	int cnt = 1;
	needlf = FALSE;
	inOP = FALSE;
	int64 cycletime;
	
	printf("Starting simple test\n \r");
    dorun = 0;
	/* initialise SOEM, bind socket to ifname */
	if (ec_init(ifname)){
		
		printf("ec_init on %s succeeded.\n \r",ifname);		 
		
		/* find and auto-config slaves */
		if ( ec_config_init(TRUE) > 0 ){
			
			 printf("%d slaves found and configured.\n \r",ec_slavecount);
			
			if((ec_slavecount >= 1))
					{
					for(slc = 1; slc <= ec_slavecount; slc++)
						{
									 // beckhoff EL7031, using ec_slave[].name is not very reliable
//							 if((ec_slave[slc].eep_man == 0x00850104) && (ec_slave[slc].eep_id == 0x01030507))
//							 {
									 printf("Found %s at position %d\n", ec_slave[slc].name, slc);
//									 u8val = 8;
//									 ec_SDOwrite(slc, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
//							 }
							ec_slave[slc].PO2SOconfig = &Servosetup;
							}
						}
//		 i=ec_config_map(&IOmap);
//		 printf("\r\nIOMAP:%d\r\n",i);
//		 ec_set_pdo_queue(ec_config_map(&IOmap),3);
						
			 ec_configdc(); //DCʱ������
			 ec_config_map(&IOmap); //添加的代码
						
			 ec_dcsync0(1, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
			 ec_dcsync0(2, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 2
       ec_dcsync0(3, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
			 ec_dcsync0(4, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 2 
       ec_dcsync0(5, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
			 ec_dcsync0(6, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 2 
       ec_dcsync0(7, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
			 ec_dcsync0(8, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 2 						
									
 			 ec_set_pdo_queue(ec_config_map(&IOmap),3);
	
//			ec_set_pdo_queue(ec_config_map(&IOmap),3);
			//ec_configdc();
			printf("Slaves mapped, state to SAFE_OP.\n \r");
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
			
			/* read indevidual slave state and store in ec_slave[] */
			ec_readstate();
						
			printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);
			printf("ec_slave[0].outputs=%d \r \n",ec_slave[0].Obytes);
						
			for(cnt = 1; cnt <= ec_slavecount ; cnt++){
				printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n \r",
					cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
					ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
			}
			
			oloop = ec_slave[0].Obytes;//��վ������վ���ֽ���������SSC-IO��2
			if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
			if (oloop > 30) oloop = 30;
			
			iloop = ec_slave[0].Ibytes;//��վ������վ���ֽ���������SSC-IO��6		
			if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
			if (iloop > 30) iloop = 30;
			
			printf("oloop:%d iloop:%d\n\r",oloop,iloop);
			
			printf("segments : %d : %d %d %d %d\n \r",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
			printf("Request operational state for all slaves\n \r");
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			printf("Calculated workcounter %d\n \r", expectedWKC);
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			
			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			/* request OP state for all slaves */
			ec_writestate(0);
			chk = 40;
			/* wait for all slaves to reach OP state */
			do{
//				ec_send_processdata();
//				ec_pdo_outframe();
				
				ec_receive_processdata(EC_TIMEOUTRET);
				ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
				ec_statecheck(1, EC_STATE_OPERATIONAL, 50000);
				
				printf("-%d %d-",ec_slave[0].state,ec_slave[1].state);
			}
			//while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
			while ( (ec_slave[0].state != EC_STATE_OPERATIONAL)||(ec_slave[1].state != EC_STATE_OPERATIONAL));
			printf("\r\n%d %d\r\n",ec_slave[0].state,ec_slave[1].state);
			if (ec_slave[0].state == EC_STATE_OPERATIONAL ){
				int pos = 0;
				int len = 0;
				printf("Operational state reached for all slaves.\n \r");
				wkc_count = 0;
				inOP = TRUE;
				
				outputs1 = (PDO_Output *)ec_slave[1].outputs;
        inputs1  = (PDO_Input *)ec_slave[1].inputs;
				
				outputs2 = (PDO_Output *)ec_slave[2].outputs;
        inputs2  = (PDO_Input *)ec_slave[2].inputs;
				
				outputs3 = (PDO_Output *)ec_slave[3].outputs;
        inputs3  = (PDO_Input *)ec_slave[3].inputs;
				
				outputs4 = (PDO_Output *)ec_slave[4].outputs;
        inputs4  = (PDO_Input *)ec_slave[4].inputs;
				
				outputs5 = (PDO_Output *)ec_slave[5].outputs;
        inputs5  = (PDO_Input *)ec_slave[5].inputs;
				
				outputs6 = (PDO_Output *)ec_slave[6].outputs;
        inputs6  = (PDO_Input *)ec_slave[6].inputs;
				
				outputs7 = (PDO_Output *)ec_slave[7].outputs;
        inputs7  = (PDO_Input *)ec_slave[7].inputs;
				
				outputs8 = (PDO_Output *)ec_slave[8].outputs;
        inputs8  = (PDO_Input *)ec_slave[8].inputs;
				
		  /* cyclic loop */
			}else{
				printf("Not all slaves reached operational state.\n \r");
				ec_readstate();
				for(i = 1; i<=ec_slavecount ; i++){
					if(ec_slave[i].state != EC_STATE_OPERATIONAL){
						printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n \r",
							i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
				printf("\nRequest init state for all slaves\n \r");
				ec_slave[0].state = EC_STATE_INIT;
				/* request INIT state for all slaves */
				ec_writestate(0);
				printf("End simple test, close socket\n \r");
        /* stop SOEM, close socket */
        ec_close();
			}           
		}
		else{
			printf("No slaves found!\n \r");
		}
	}else{
		printf("No socket connection on %s\nExcecute as root\n \r",ifname);
	} 
	
	 dorun = 1;
	 cycletime = SYNC0TIME * 1000;
	 i=0;
			
	while(1){	
		//void *ptr ;
		if (dorun>0){
			if(pdoTimeFlag == 1){
				pdoTimeFlag = 0;
				i++;
			}
		}
		
		else{
			printf("\nRequest init state for all slaves\n \r");
			ec_slave[0].state = EC_STATE_INIT;
			/* request INIT state for all slaves */
			ec_writestate(0);
			printf("End simple test, close socket\n \r");
			/* stop SOEM, close socket */
			ec_close();
		}
	}	
	
} 

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
//	soft_reset_do();
	
	uint8_t flag=0;
	int64 toff, gl_delta;
	
	dorun = 0;
	
	/* ��ʼ��LED */
	LED_GPIO_Config();
	
	/* ��ʼ������ */
	Key_GPIO_Config();
	
	
	/* 调试USART初始化 */
	Debug_USART_Config();
	
	/* RS485初始化 */
  RS485_Init(19200);		
	
	/* ��ʼ��ϵͳ�δ�ʱ�� */	
	SysTick_Init();
	
	TIM8_Cap_Init(999,168-1); //1ms
	
  TIM3_Config(SYNC0TIME,84-1); //1ms

	/* Configure ethernet (GPIOs, clocks, MAC, DMA) */
  ETH_BSP_Config();	
	
  printf("LAN8720A BSP INIT AND COMFIGURE SUCCESS\n \r");
	
  ethernetif_init();
	
	osal_usleep(50000);
	
  simpletest("eth0");
	
	    while(1){
		   
//           RS485_Receive_Data();	
	     };
			
	return 0;
			 
}

/**
  * @brief  ͨ�ö�ʱ��3�жϳ�ʼ��
  * @param  period : �Զ���װֵ��
  * @param  prescaler : ʱ��Ԥ��Ƶ��            1000          900
  * @retval ��                             100000        9
  * @note   ��ʱ�����ʱ����㷽��:Tout=((period+1)*(prescaler+1))/Ft us.
  *          Ft=��ʱ������Ƶ��,ΪSystemCoreClock/2=90,��λ:Mhz
  */
static void TIM3_Config(uint16_t period,uint16_t prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=84 - 1;//prescaler;//9 - 1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=1000 - 1 ;//period;//10000 - 1;   //10ms
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0; //��ռ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
}

/**
  * @brief  ��ʱ��3�жϷ�����
  * @param  ��
  * @retval ��
  */
void TIM3_IRQHandler(void)
{ 
	
		if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
		{
			app_time_base += SYNC0TIME*1000;
			pdoTimeFlag = 1;
			
			if (dorun==0){	
				ec_send_processdata();//��վ���ݷ���
				ec_pdo_outframe();
			}
			
			else {
				
				ecat_app();
				
			}
			//LED2=!LED2;
			  LocalTime += 10;//10ms����
		}
	  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
