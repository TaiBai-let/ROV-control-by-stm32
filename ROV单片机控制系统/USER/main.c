/*
 *                             _ooOoo_
 *                            o8888888o
 *                            88" . "88
 *                            (| -_- |)
 *                            O\  =  /O
 *                         ____/`---'\____
 *                       .'  \\|     |//  `.
 *                      /  \\|||  :  |||//  \
 *                     /  _||||| -:- |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|  ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. /
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"".
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=---='
 *          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                    佛祖保佑        一切如愿
*/
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "pid.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "lcd.h"
#include "control.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
 

int ROVmode; 
float pitch,roll,yaw; 		//欧拉角
	
/***************************************************************************/
/*                              MPU6050姿态解算                            */	
void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
 	

//MPU6050姿态解算传并给上位机
 int main(void)
 {	
	 
	PID_init();
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(500000);	 	//串口初始化为500000
	delay_init();	//延时初始化 
	usmart_dev.init(72);		//初始化USMART
	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD  
	MPU_Init();					//初始化MPU6050
 	POINT_COLOR=RED;			//设置字体为红色 
	LCD_ShowString(30,50,200,16,16,"STM32");	
	LCD_ShowString(30,70,200,16,16,"MPU6050 PWM PID");	
	LCD_ShowString(30,90,200,16,16,"RUN");
	LCD_ShowString(30,110,200,16,16,"2020/2/4");	

	LCD_ShowString(30,130,200,16,16,"MPU6050 OK");
	LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
	POINT_COLOR=BLUE;//设置字体为蓝色 
 	LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");	 
 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
 	LCD_ShowString(30,220,200,16,16,"Pitch:    . C");	
 	LCD_ShowString(30,240,200,16,16," Roll:    . C");	 
 	LCD_ShowString(30,260,200,16,16," Yaw :    . C");	 
	
	
	
	PID_init(); //PID初始化定义

	
	/***************************************************************************/
	/*                       PWM初始化&&解锁电调                               */
	TIM3_PWM_Init(1999,71);     //设置为周期2ms
	TIM8_PWM_Init(1999,71);
	TIM2_PWM_Init(1999,71);
	
	
	/*解锁电调，电调第一次使用时设置行程*/
	/*全为向上计数且高电平有效，此时电调正向*/
	TIM_SetCompare1(TIM3,10);
	TIM_SetCompare2(TIM3,10);
  TIM_SetCompare1(TIM8,10);
  TIM_SetCompare2(TIM8,10);
  TIM_SetCompare2(TIM2,10);
  TIM_SetCompare3(TIM2,10);	 
	delay_ms(1500);       //根据电调使用说明设置
	TIM_SetCompare1(TIM3,500);
	TIM_SetCompare2(TIM3,500);
  TIM_SetCompare1(TIM8,500);
  TIM_SetCompare2(TIM8,500);
  TIM_SetCompare2(TIM2,500);
  TIM_SetCompare3(TIM2,500);	
  delay_ms(1000);	
	
	
	//TIM7用于定时采集数据进行pid算法
	time7_init(71,9999);//每10ms采集pid进行一次计算
  NVIC_INIT();
	/***************************************************************************/	
	/*                               主控函数                                  */
	
	while(1)
{
	u8 t=0,report=1;			//默认开启上报
	u8 key;
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
	
	
	//设置比例系数
	PIDSetKp(12,12,12);
	PIDSetKi(2,2,2);
	PIDSetKd(2,2,2);
		
	while(mpu_dmp_init())
 	{
		LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
		delay_ms(200);
		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(200);
	}  
	
  /***************************************************************************/
	/*                              姿态解算                                   */
		key=KEY_Scan(0);
	
		if(key==KEY0_PRES)
		{
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}
		
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			if((t%10)==0)
			{ 
				if(temp<0)
				{
					LCD_ShowChar(30+48,200,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,200,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//显示小数部分 
				temp=pitch*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,220,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,220,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,220,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//显示小数部分 
				temp=roll*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,240,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,240,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//显示小数部分 
				temp=yaw*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,260,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,260,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//显示小数部分  
				t=0;
				LED0=!LED0;//LED闪烁
			}
		}
		t++; 
		
		
/***************************************************************************/		
/*                               机械臂的控制                              */	

 TIM1_Init();    //初始化TIM1

 static int MAmode=5;  //记录此时机械臂舵机所在位置
 int reset=0;          //定义按键未按下的状态
 int control1=reset;   //加紧按键 未按下
 int control2=reset;   //松开按键 未按下
				
	if(control1!=reset&&MAmode!=9)    //判断是否为 加紧控制
{  MAmode++; }                                                 //且在最大最小状态时，不进一步控制
 else if(control2!=reset&&MAmode!=1)  //判断是否为 松开控制
{  MAmode--; }          
 
   switch(MAmode)
	{
			case(1):
		{
		TIM_SetCompare1(TIM1, 1000);
    delay_ms(500);
		 }   break;
				 
		 	case(2):
		{
		TIM_SetCompare1(TIM1, 1125);
    delay_ms(500);
		 }   break;
				 
		 	case(3):
		{
		TIM_SetCompare1(TIM1, 1250);
    delay_ms(500);
		 }   break;
				 
		 case(4):
		{
		TIM_SetCompare1(TIM1, 1375);
    delay_ms(500);
		 }   break;

		 case(5):
		{
		TIM_SetCompare1(TIM1, 1500);  //初始状态下，为1500即为回到中间
    delay_ms(500);
		 }   break;
		 
		 case(6):
		{
		TIM_SetCompare1(TIM1, 1625);
    delay_ms(500);
		 }   break;

		 case(7):
		{
		TIM_SetCompare1(TIM1, 1750);
    delay_ms(500);
		 }   break;

		 case(8):
		{
		TIM_SetCompare1(TIM1, 1875);
    delay_ms(500);
		 }   break;

		 case(9):
		{
		TIM_SetCompare1(TIM1, 2000);
    delay_ms(500);
		 }   break;
		 
		 default:  break;
	}


	
/***************************************************************************/		
/*                               云台的控制                                */	
	
 TIM1_Init();     //初始化TIM1
 
 static int HDmode=9;  //记录此时云台舵机所在位置
 int control3=reset;   //抬升按键 未按下
 int control4=reset;   //下降按键 未按下

	if(control3!=reset&&HDmode!=17)    //判断是否为 抬升控制
{  HDmode++; }                                                 //且在最大最小状态时，不进一步控制
 else if(control4!=reset&&HDmode!=1)  //判断是否为 下降控制
{  HDmode--; }          
 
   switch(HDmode)
	{
			case(1):
		{
		TIM_SetCompare2(TIM1, 1020);
    delay_ms(300);
		 }   break;
				 
		 	case(2):
		{
		TIM_SetCompare2(TIM1, 1080);
    delay_ms(300);
		 }   break;
				 
		 	case(3):
		{
		TIM_SetCompare2(TIM1, 1140);
    delay_ms(300);
		 }   break;
				 
		 case(4):
		{
		TIM_SetCompare2(TIM1, 1200);
    delay_ms(300);
		 }   break;

		 case(5):
		{
		TIM_SetCompare2(TIM1, 1260);
    delay_ms(300);
		 }   break;
		 
		 case(6):
		{
		TIM_SetCompare2(TIM1, 1320);
    delay_ms(300);
		 }   break;

		 case(7):
		{
		TIM_SetCompare2(TIM1, 1380);
    delay_ms(300);
		 }   break;

		 case(8):
		{
		TIM_SetCompare2(TIM1, 1440);
    delay_ms(300);
		 }   break;

		 case(9):
		{
		TIM_SetCompare2(TIM1, 1500);
    delay_ms(300);
		 }   break;
		 
		 case(10):
		{
		TIM_SetCompare2(TIM1, 1560);
    delay_ms(300);
		 }   break;
				 
		 	case(11):
		{
		TIM_SetCompare2(TIM1, 1620);
    delay_ms(300);
		 }   break;
				 
		 	case(12):
		{
		TIM_SetCompare2(TIM1, 1680);
    delay_ms(300);
		 }   break;
				 
		 case(13):
		{
		TIM_SetCompare2(TIM1, 1740);
    delay_ms(300);
		 }   break;

		 case(14):
		{
		TIM_SetCompare2(TIM1, 1800);
    delay_ms(300);
		 }   break;
		 
		 case(15):
		{
		TIM_SetCompare2(TIM1, 1860);
    delay_ms(300);
		 }   break;

		 case(16):
		{
		TIM_SetCompare2(TIM1, 1920);
    delay_ms(300);
		 }   break;

		 case(17):
		{
		TIM_SetCompare2(TIM1, 1980);
    delay_ms(300);
		 }   break;
		 default:  break;
	}


	/***************************************************************************/
	/*                          抗饱和pid算法                                  */	
	ROVmode=1;
	ROV_Control();          //控制函数		
 }
}

	