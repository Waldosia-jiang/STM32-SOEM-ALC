#include "timer.h"
#include "RS485.h"
#include "bsp_MB_host.h"

#define PI  3.14159265358979f

volatile uint8_t DATA_10H[64] = {0x03,0xE8,0x0B,0xB8,0x05,0xDC,0x27,0x10,0x03,0xE8,0x88,0x13};

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
uint8  TIM8CH1_CAPTURE_STA = 0;	
uint16 TIM8CH1_CAPTURE_UPVAL;
uint16 TIM8CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志	
uint8  TIM8CH2_CAPTURE_STA = 0;		
uint16 TIM8CH2_CAPTURE_UPVAL;
uint16 TIM8CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志	
uint8  TIM8CH3_CAPTURE_STA = 0;		
uint16 TIM8CH3_CAPTURE_UPVAL;
uint16 TIM8CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志
uint8  TIM8CH4_CAPTURE_STA = 0;			
uint16 TIM8CH4_CAPTURE_UPVAL;
uint16 TIM8CH4_CAPTURE_DOWNVAL;

uint32 TIM8_T1;
uint32 TIM8_T2;
uint32 TIM8_T3;
uint32 TIM8_T4;

//循环发送03命令读取数据标志
int FLAG_03 = 0;

//循环发送10命令写数据标志
int FLAG_10 = 0;

//循环发送06命令写数据标志
//int FLAG_06 = 0;

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s
float RC_Velocity=500; 

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
volatile float Move_X, Move_Y, Move_Z;

//高速、低速参数(底盘)
volatile float vel = 0.3;

//高速、低速参数(液压)
volatile float vel_yy = 0.75;
// 10H指令发送标志位
volatile uint8_t  RS485_10H_FLAG = 5; 

//Variables related to remote control acquisition of model aircraft
//航模遥控采集相关变量
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//航模遥控接收变量
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  

///**************************************************************************
//Function: Model aircraft remote control initialization function, timer 1 input capture initialization
//Input   : arr: Automatic reload value, psc: clock preset frequency
//Output  : none
//函数功能：航模遥控初始化函数，定时器8输入捕获初始化
//入口参数：arr：自动重装值，psc：时钟预分频数 
//返 回 值：无
//**************************************************************************/ 
void TIM8_Cap_Init(uint16 arr, uint16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8时钟使能  	

	/*** Initialize timer 1 || 初始化定时器1 ***/
	//Set the counter to automatically reload //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //预分频器 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM向上计数模式	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

	
	//Interrupt priority group setting
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /*** interrupt packet initialization || 中断分组初始化 ***/
    //TIM8 interrupts //TIM8中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn; 
   //Preempt priority 0 //先占优先级0级	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
	//Level 0 from priority //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	//IRQ channels are enabled //IRQ通道被使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	NVIC_Init(&NVIC_InitStructure);   
	
	//Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts,allowed update_interrupts
	//允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	
	TIM_ITConfig(TIM8, TIM_IT_Update,	ENABLE);
	//Advanced timer output must be enabled //高级定时器输出必须使能这句	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	//Enable timer //使能定时器
	TIM_Cmd(TIM8, ENABLE);
	
}
///**************************************************************************
//Function: Model aircraft remote control receiving interrupt, namely timer 1 input capture interrupt
//Input   : none
//Output  : none
//函数功能：航模遥控接收中断，即定时器1输入捕获中断
//入口参数：无
//返 回 值：无
//**************************************************************************/ 
//void TIM8_CC_IRQHandler(void){
//	
//	for(int i = 0 ;i < 6;i++){
//	   printf("RS485_RX_BUF[%d] = %x \n", i,RS485_RX_BUF[i]);
//	 }
//  printf("\n \r");
//	
//  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);	 
//}

void TIM8_CC_IRQHandler(void)
{ 
   	
	static uint8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	
	//连接航模遥遥控器后，需要推下前进杆，才可以正式航模控制小车
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
//  if(Remoter_Ch2>1600&&Remote_ON_Flag==0&&Deviation_Count>=CONTROL_DELAY)
//  {
//		//Model aircraft remote control mark position 1, other marks position 0
//		//航模遥控标志位置1，其它标志位置0
//		Remote_ON_Flag=1;
//	  APP_ON_Flag=0;
//		PS2_ON_Flag=0;
//		CAN_ON_Flag=0;
//		Usart_ON_Flag=0; 
//	}
//	 
	//Channel 1 //通道一
	if ((TIM8CH1_CAPTURE_STA & 0X80) == 0) 			
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) //A capture event occurred on channel 1 //通道1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); //Clear the interrupt flag bit //清除中断标志位
			if (TIM8CH1_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
			{
				TIM8CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM8); //Record the timer value at this point //记录下此时的定时器计数值
				if (TIM8CH1_CAPTURE_DOWNVAL < TIM8CH1_CAPTURE_UPVAL)
				{
					TIM8_T1 = 9999;
				}
				else
					TIM8_T1 = 0;
				Remoter_Ch1 = TIM8CH1_CAPTURE_DOWNVAL - TIM8CH1_CAPTURE_UPVAL + TIM8_T1;	//Time to get the total high level //得到总的高电平的时间
				
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if( ch1_filter_times<=5 ) Remoter_Ch1=L_Remoter_Ch1; //Filter //滤波	
					else ch1_filter_times=0;
				}
				else
				{
					ch1_filter_times=0;
				}
				 L_Remoter_Ch1=Remoter_Ch1;
				
				TIM8CH1_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
				TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM8CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM8); //Obtain rising edge data //获取上升沿数据
				TIM8CH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
			}
		}
	}
	
  //Channel 2 //通道二
	if ((TIM8CH2_CAPTURE_STA & 0X80) == 0)		
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)	//A capture event occurred on channel 2 //通道2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC2); //Clear the interrupt flag bit //清除中断标志位
			if (TIM8CH2_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
			{
				TIM8CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM8); //Record the timer value at this point //记录下此时的定时器计数值
				if (TIM8CH2_CAPTURE_DOWNVAL < TIM8CH2_CAPTURE_UPVAL)
				{
					TIM8_T2 = 9999;
				}
				else
					TIM8_T2 = 0;
				Remoter_Ch2 = TIM8CH2_CAPTURE_DOWNVAL - TIM8CH2_CAPTURE_UPVAL + TIM8_T2; //Time to get the total high level //得到总的高电平的时间
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if( ch2_filter_times<=5 ) Remoter_Ch2=L_Remoter_Ch2; //Filter //滤波	
					else ch2_filter_times=0;
				}
				else
				{
					ch2_filter_times=0;
				}
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM8CH2_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM8CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM8); //Obtain rising edge data //获取上升沿数据
				TIM8CH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
			}
		}
	}
  //Channel 3 //通道三
	if ((TIM8CH3_CAPTURE_STA & 0X80) == 0)			
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)	//A capture event occurred on channel 3 //通道3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC3); //Clear the interrupt flag bit //清除中断标志位
			if (TIM8CH3_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
			{
				TIM8CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM8); //Record the timer value at this point //记录下此时的定时器计数值
				if (TIM8CH3_CAPTURE_DOWNVAL < TIM8CH3_CAPTURE_UPVAL)
				{
					TIM8_T3 = 9999;
				}
				else
					TIM8_T3 = 0;
				Remoter_Ch3 = TIM8CH3_CAPTURE_DOWNVAL - TIM8CH3_CAPTURE_UPVAL + TIM8_T3; //Time to get the total high level //得到总的高电平的时间
				if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)
				{
					ch3_filter_times++;
					if( ch3_filter_times<=5 ) Remoter_Ch3=L_Remoter_Ch3; //Filter //滤波	
					else ch3_filter_times=0;
				}
				else
				{
					ch3_filter_times=0;
				}
				L_Remoter_Ch3=Remoter_Ch3;
				TIM8CH3_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
				TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM8CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM8); //Obtain rising edge data //获取上升沿数据
				TIM8CH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
			}
		}
	}

		//Channel 4 //通道四
		if ((TIM8CH4_CAPTURE_STA & 0X80) == 0)		
		{
			if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)	//A capture event occurred on channel 4 //通道4发生捕获事件
			{
				TIM_ClearITPendingBit(TIM8, TIM_IT_CC4); //Clear the interrupt flag bit //清除中断标志位
				if (TIM8CH4_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
				{
					TIM8CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM8); //Record the timer value at this point //记录下此时的定时器计数值
					if (TIM8CH4_CAPTURE_DOWNVAL < TIM8CH4_CAPTURE_UPVAL)
					{
						TIM8_T4 = 9999;
					}
					else
						TIM8_T4 = 0;
					Remoter_Ch4 = TIM8CH4_CAPTURE_DOWNVAL - TIM8CH4_CAPTURE_UPVAL + TIM8_T4; //Time to get the total high level //得到总的高电平的时间
					if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)
					{
						ch4_filter_times++;
						if( ch4_filter_times<=5 ) Remoter_Ch4=L_Remoter_Ch4; //Filter //滤波	
						else ch4_filter_times=0;
					}
					else
					{
						ch4_filter_times=0;
					}
					L_Remoter_Ch4=Remoter_Ch4;				
					TIM8CH4_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
					TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
				}
				else 
				{
					//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				  //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
					TIM8CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM8); //Obtain rising edge data //获取上升沿数据
					TIM8CH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
					TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
				}
			}
		}
		
		Remote_Control();
		
		printf(" Move_X= = %f , Move_Y= %f , Move_Z= %f ,", Move_X , Move_Y, Move_Z);
//		printf(" %f ", Move_Z);
}

/**************************************************************************
Function: TIM8 Update Interrupt
Input   : none
Output  : none
函数功能：定时器8更新中断
入口参数：无
返回  值：无 
**************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void) 
{ 
//Clear the interrupt flag bit
//清除中断标志位 
//  TIM8->SR&=~(1<<0);
	 if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
 	{
	   TIM_ClearITPendingBit(TIM8,TIM_IT_Update);	  //清除TIMx的中断待处理位:TIM 中断源
		 if(time_start != 0)//运行时间 !=0 表明
 		 {
 		     timout++;
 		  if(timout >=30)
 		  {
 		     time_start = 0; //停止计时
 			   Rx_flag = 1;    //接收数据完毕
 		  }
 			
 		 }

//	for(int i = 0 ;i < 6;i++){
//	   printf("RS485_RX_BUF[%d] = %x \n", i,RS485_RX_BUF[i]);
//	 }
//  printf("\n \r");
	
		if(FLAG_03++ == 100){
			 
			 MB_ReadHoldingReg_03H( 0x05, 0, 4);
			 FLAG_03 = 0;	
		}
	  
		if( FLAG_10++ == 1000){
//		if( RS485_10H_FLAG-- > 0 ){
			
//		 printf("vel_yy = %f \n \r", vel_yy);
			
       uint16 temp1 = 0x03E8;//平移
			 DATA_10H[0] = (uint16)((temp1) * (vel_yy)) >> 8;
			 DATA_10H[1] = (temp1) * (vel_yy) ;
			
			 uint16 temp2 = 0x0BB8;//上下
			 DATA_10H[2] = (uint16)((temp2) * (vel_yy)) >> 8;
			 DATA_10H[3] = (temp2) * (vel_yy) ;
			
			 uint16 temp3 = 0x05DC;//俯仰
			 DATA_10H[4] = (uint16)((temp3) * (vel_yy)) >> 8;
			 DATA_10H[5] = (temp3) * (vel_yy);
			 
			 uint16 temp5 = 0x07D0;//旋转
			 DATA_10H[8] = (uint16)((temp5) * (vel_yy)) >> 8;
			 DATA_10H[9] = (temp5) * (vel_yy);
			 
			 MB_WriteNumHoldingReg_10H( 0x01, 10, 6 , (uint8_t *)DATA_10H);

//			 if(RS485_10H_FLAG == 1) RS485_10H_FLAG = 0;
			 
			 FLAG_10 = 0;		
		}
		
//		// 物联网模块
//		if( FLAG_06++ == 10000){	 
//			 
//			 MB_WriteHoldingReg_06H( 0x06, 1 , RS485_06H);

////			 if(RS485_10H_FLAG == 1) RS485_10H_FLAG = 0;
//			 
//			 FLAG_06 = 0;		
//		}
	
		 if(Rx_flag == 1)  RS485_Receive_Data();
			
		 } 
  }
/**************************************************************************
Function: TIM8 Update Interrupt
Input   : none
Output  : none
函数功能：限幅函数
入口参数：无
返回  值：无 
**************************************************************************/
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static uint8 thrice=100;
    int Threshold=100;

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 
//	  static float Target_LX,Target_LY,Target_RY,Target_RX;
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

		// Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500;
	
//		//Left joystick left and right. Control left and right movement.
	  //左摇杆左右方向。控制左右移动。。
    LY=Remoter_Ch4-1500;
	
		//Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;		//
	
    //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;
		
		
//		if(LX==0) Target_LX=Target_LX/1.2f;
//		if(LY==0) Target_LY=Target_LY/1.2f;
//		if(RY==0) Target_RY=Target_RY/1.2f;
		
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX; 
		Move_Y= LY;
		Move_Z= RY; 
    Move_X= Move_X*Remote_RCvelocity/500; 
		Move_Y= Move_Y*Remote_RCvelocity/500;
		Move_Z= Move_Z*(PI/4)/500;  
			 
		//Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;
		
		//Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
			
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析			
}
