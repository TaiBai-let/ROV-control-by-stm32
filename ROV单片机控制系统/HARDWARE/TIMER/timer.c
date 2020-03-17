#include "timer.h"
#include "led.h"
#include "usart.h"
#include "pid.h"
#include "mpu6050.h"

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//使用的是定时器3
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
  TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);   //使能指定的TIM3中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能 
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设			 
}

//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIO外设 
 
   //输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //TIM_CH1&CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode= TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式为输出比较模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式为输出比较模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

  TIM_CtrlPWMOutputs(TIM3,ENABLE);	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	TIM_ARRPreloadConfig(TIM3, ENABLE); 
}

//定时器3中断服务程序
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );
		}
}



//通用定时器8中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//使用的是定时器8
void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

                                                             
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); 

  TIM_CtrlPWMOutputs(TIM8,ENABLE);	

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  

	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	TIM_Cmd(TIM8, ENABLE); 
}


//定时器8中断服务程序
void TIM8_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
		{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update  );
		}
}



//通用定时器2中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//使用的是定时器2
void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

                                                             
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure); 

  TIM_CtrlPWMOutputs(TIM2,ENABLE);	

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  

	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	TIM_Cmd(TIM8, ENABLE); 
}


//定时器8中断服务程序
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );
		}
}



//TIM7用于给PID控制提供计时器
//pid定期捕获
extern float pitch,roll,yaw; 		//欧拉角
extern struct _pid pid_roll,pid_pitch,pid_yaw;

void time7_init(u16 per,u16 pre)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=per;
    TIM_TimeBaseStructure.TIM_Prescaler=pre;
    TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);

    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM7,ENABLE);
}
void NVIC_INIT(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
    {
        TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    }
		
		pid_roll.Actualangle=roll;     //记录姿态角
		pid_pitch.Actualangle=pitch;
		pid_yaw.Actualangle=yaw;
		
		roll_pitch_yaw_anglePID(0,0,0);  //计算pid控制后的输出值
}



/*****************************************************************/
/*     如果有需要，可以TIM1输出互补PWM，下面为配置TIM1的代码     */
//初始化所用GPIO口
//static void TIM1_GPIO_Config(void) 
//{
//  GPIO_InitTypeDef GPIO_InitStructure;

//  /* TIM1 clock enable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

//  /* GPIOA and GPIOB clock enable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
//    
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//}

////初始化定时器功能配置
//u16 CCR2_Val = 500;
//u16 CCR3_Val = 500;//占空比

///* 配置TIM1输出PWM的模式 */
//void TIM1_Mode_Config(void)
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
//    TIM_OCInitTypeDef        TIM_OCInitStructure;

//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 1000-1; 
//    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//1ms
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//与死区分频有关
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//    
//    /****** 配置BDTR寄存器，配置死区时间******/
//    /*
//       ????? 72M   TIM_ClockDivision = TIM_CKD_DIV1?,  Tdts = 13.89ns
//       0 - 1.764us  ????
//       1.778us - 3.505us  ????
//       3.556us - 7.000us  ???? 
//       7.1117us - 14us    ????
//       ??????,??TIM_ClockDivision??
//    */
//    TIM1_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
//    TIM1_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
//    TIM1_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
//    TIM1_BDTRInitStruct.TIM_DeadTime = 205; //????  72:1us 172:3us 205:5us
//    TIM_BDTRConfig(TIM1,&TIM1_BDTRInitStruct);
// 
////    TIM1->BDTR |= 72;   //设置死区
//    
//     /* PWM1 Mode configuration: Channel2 */
//     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出功能
//     TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//比较互补输出使能
//     TIM_OCInitStructure.TIM_Pulse = CCR2_Val;  
//     TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
//     TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//互补输出极性
//     TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//空闲状态下输出比较的引脚状态
//     TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
//     TIM_OC2Init(TIM1, &TIM_OCInitStructure);  
//     TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);      //自动重装载使能


//    /* PWM1 Mode configuration: Channel3 */
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;      
//    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);    

//    TIM_ARRPreloadConfig(TIM1, ENABLE);//立即生效

//    /* TIM1 enable counter */
//    TIM_Cmd(TIM1, ENABLE);
//    
//    TIM_CtrlPWMOutputs(TIM1, ENABLE);  
//}

