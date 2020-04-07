#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "pid.h"
#include "control.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"


//声明所用变量
extern int ROVmode;
//float PID_dt;
extern struct _pid pid_roll,pid_pitch,pid_yaw;
extern float pitch,roll,yaw; 		//欧拉角

//运动控制
void ROV_Control(void)
{

		switch(ROVmode)           //选择运动模式
	{
		case 1:        //前进
		{
    PWM_Forward_OUT();	 		
    } break;

		case 2:       //后退
	  {		
     PWM_Back_OUT();
    }break;

		case 3:       //左转
	  {		
		 PWM_Left_OUT();
    }break;

		case 4:       //右转
	  {		
		 PWM_Right_OUT();
    }break;

		case 5:       //上升
	  {		
     PWM_Up_OUT();
    }break;
	
		case 6:       //下降
	  {		
     PWM_Down_OUT();
    }break;
		
		case 7:       //左移
		{		
     PWM_Leftmove_OUT();
    }break;
		
		case 8:       //右移
		{		
     PWM_Rightmove_OUT();
    }break;
		
		case 9:       //左翻
		{		
     PWM_Leftturn_OUT();
    }break;
		
		case 10:      //右翻
		{		
     PWM_Rightturn_OUT();
    }break;
		
		default : break;
		
	}
}



//---------------计算PWM输出------------------------------------ 

//前进
void PWM_Forward_OUT(void)	 			 //TIM3和TIM8工作
{	
			static int yaw_go_left1=100;
			static int yaw_go_left2=1500;
			static int yaw_go_right1=100;
			static int yaw_go_right2=1500;
		/*TIM3的定时器控制前两正桨正转*/
      TIM_SetCompare1(TIM3,yaw_go_left1);
	    TIM_SetCompare2(TIM3,yaw_go_right1);
		/*TIM8的定时器控制后两反桨反转*/
	    TIM_SetCompare1(TIM8,yaw_go_left2);
	    TIM_SetCompare2(TIM8,yaw_go_right2);
	
	do{
		  roll_pitch_yaw_anglePID(0,0,0);
		if(pid_yaw.Actualangle<-200)
		{
			//航行向左偏移，需要左部推力增大
			yaw_go_left1+=pid_yaw.control;
			yaw_go_left2+=pid_yaw.control;
		  TIM_SetCompare1(TIM3,yaw_go_left1);    //头部左侧加速
	    TIM_SetCompare1(TIM8,yaw_go_left2);    //尾部左侧加速
		}
		else if(pid_yaw.Actualangle>+200)
		{
			//航行向右偏移，需要右部推力增大
			yaw_go_right1+=pid_yaw.control;
			yaw_go_right2+=pid_yaw.control;
		  TIM_SetCompare2(TIM3,yaw_go_right1);    //头部右侧加速
	    TIM_SetCompare2(TIM8,yaw_go_right2);    //尾部右侧加速		
		}
	}while(-200<pid_yaw.Actualangle<+200);
}


//后退
void PWM_Back_OUT(void)
{
			static int yaw_back_right1=1500;
			static int yaw_back_right2=100;
			static int yaw_back_left1=1500;
			static int yaw_back_left2=100;
		/*TIM3的定时器控制前两正桨反转*/
      TIM_SetCompare1(TIM3,yaw_back_left1);
	    TIM_SetCompare2(TIM3,yaw_back_right1);
		/*TIM8的定时器控制前两反桨正转*/
	 	  TIM_SetCompare1(TIM8,yaw_back_left2);
	    TIM_SetCompare2(TIM8,yaw_back_right2); 	
	
	
	do{
		  roll_pitch_yaw_anglePID(0,0,0);
		if(pid_yaw.Actualangle<-200)
		{
			//航行向右偏移，需要右部推力增大
      yaw_back_right1+=pid_yaw.control;
			yaw_back_right2+=pid_yaw.control;
		  TIM_SetCompare1(TIM3,yaw_back_right1);    //头部左侧加速
	    TIM_SetCompare1(TIM8,yaw_back_right2);    //尾部左侧加速
		}
		else if(pid_yaw.Actualangle>+200)
		{
			//航行向左偏移，需要左部推力增大
      yaw_back_left1+=pid_yaw.control;
			yaw_back_left2+=pid_yaw.control;
		  TIM_SetCompare2(TIM3,yaw_back_left1);    //头部右侧加速
	    TIM_SetCompare2(TIM8,yaw_back_left2);    //尾部右侧加速		
		}
	}while(-200<pid_yaw.Actualangle<+200);
}


//上升
void PWM_Up_OUT(void)
{
			static int roll_up_left=1500;
			static int roll_up_right=100;

      TIM_SetCompare2(TIM2,roll_up_left);
      TIM_SetCompare3(TIM2,roll_up_right);
	
	
	do{
		roll_pitch_yaw_anglePID(0,0,0);
		if(pid_roll.Actualangle<-200)
		{
			//航行向左倾斜，需要左部上升推力增大
      roll_up_left+=pid_roll.control;
			roll_up_right-=pid_roll.control;
		  TIM_SetCompare2(TIM2,roll_up_left);    //垂直左侧加速
	    TIM_SetCompare3(TIM2,roll_up_right);    //垂直右侧减速
		}
		else if(pid_roll.Actualangle>+200)
		{
			//航行向右倾斜，需要右部上升推力增大
			roll_up_left-=pid_roll.control;
      roll_up_right+=pid_roll.control;
		  TIM_SetCompare2(TIM2,roll_up_left);    //垂直左侧减速
	    TIM_SetCompare3(TIM2,roll_up_right);    //垂直右侧加速
		}
	}while(-200<pid_roll.Actualangle<+200);
}


//下降
void PWM_Down_OUT(void)
{
			static int roll_down_left=100;
			static int roll_down_right=1500;
      TIM_SetCompare2(TIM2,roll_down_left);
      TIM_SetCompare3(TIM2,roll_down_right);
	
	
	do{
		  roll_pitch_yaw_anglePID(0,0,0);
		if(pid_roll.Actualangle<-200)
		{
			//航行向左倾斜，需要左部下降推力减小
      roll_down_left-=pid_roll.control;
			roll_down_right+=pid_roll.control;
		  TIM_SetCompare2(TIM2,roll_down_left);    //垂直左侧减速
	    TIM_SetCompare3(TIM2,roll_down_right);    //垂直右侧加速
		}
		else if(pid_roll.Actualangle>+200)
		{
			//航行向右倾斜，需要右部下降推力减小
      roll_down_left+=pid_roll.control;
			roll_down_right-=pid_roll.control;
		  TIM_SetCompare2(TIM2,roll_down_left);    //垂直左侧加速
	    TIM_SetCompare3(TIM2,roll_down_right);    //垂直右侧减速
		}
	}while(-200<pid_roll.Actualangle<+200);
}


//左转
void PWM_Left_OUT(void)
{
	TIM_SetCompare1(TIM3,100);
	TIM_SetCompare2(TIM3,1500);
  TIM_SetCompare1(TIM8,1500);
  TIM_SetCompare2(TIM8,100);
}


//右转
void PWM_Right_OUT(void)
{
	TIM_SetCompare1(TIM3,1500);
	TIM_SetCompare2(TIM3,100);
  TIM_SetCompare1(TIM8,100);
  TIM_SetCompare2(TIM8,1500);
}


//左移
void PWM_Leftmove_OUT(void)	 			 //TIM3和TIM8工作
{	
			static int yaw_leftmove_left1=100;
			static int yaw_leftmove_left2=100;
			static int yaw_leftmove_right1=1500;
			static int yaw_leftmove_right2=1500;

      TIM_SetCompare1(TIM3,yaw_leftmove_left1);
	    TIM_SetCompare2(TIM3,yaw_leftmove_right1);

	    TIM_SetCompare1(TIM8,yaw_leftmove_left2);
	    TIM_SetCompare2(TIM8,yaw_leftmove_right2);
	
	do{
		  roll_pitch_yaw_anglePID(0,0,0);
		if(pid_yaw.Actualangle<-200)
		{
			//航行向左偏移，需要左部推力增大
			yaw_leftmove_left1-=pid_yaw.control;
			yaw_leftmove_right1-=pid_yaw.control;
		  TIM_SetCompare1(TIM3,yaw_leftmove_left1);    //头部左侧减速
	    TIM_SetCompare2(TIM3,yaw_leftmove_right1);    //头部右侧减速
		}
		else if(pid_yaw.Actualangle>+200)
		{
			//航行向右偏移，需要右部推力增大
			yaw_leftmove_left2-=pid_yaw.control;
			yaw_leftmove_right2-=pid_yaw.control;
		  TIM_SetCompare1(TIM8,yaw_leftmove_left2);    //尾部左侧减速	
	    TIM_SetCompare2(TIM8,yaw_leftmove_right2);    //尾部右侧减速		
		}
	}while(-200<pid_yaw.Actualangle<+200);
}

//右移
void PWM_Rightmove_OUT(void)	 			 //TIM3和TIM8工作
{	
			static int yaw_rightmove_left1=1500;
			static int yaw_rightmove_left2=1500;
			static int yaw_rightmove_right1=100;
			static int yaw_rightmove_right2=100;

      TIM_SetCompare1(TIM3,yaw_rightmove_left1);
	    TIM_SetCompare2(TIM3,yaw_rightmove_right1);

	    TIM_SetCompare1(TIM8,yaw_rightmove_left2);
	    TIM_SetCompare2(TIM8,yaw_rightmove_right2);
	
	do{
		  roll_pitch_yaw_anglePID(0,0,0);
		if(pid_yaw.Actualangle<-200)
		{
			//航行向左偏移，需要左部推力增大
			yaw_rightmove_left2-=pid_yaw.control;
			yaw_rightmove_right2-=pid_yaw.control;
		  TIM_SetCompare1(TIM3,yaw_rightmove_left2);      //尾部左侧减速	
	    TIM_SetCompare2(TIM3,yaw_rightmove_right2);     //尾部右侧减速	
		}
		else if(pid_yaw.Actualangle>+200)
		{
			//航行向右偏移，需要右部推力增大
			yaw_rightmove_left1-=pid_yaw.control;
			yaw_rightmove_right1-=pid_yaw.control;
		  TIM_SetCompare1(TIM8,yaw_rightmove_left1);     //头部左侧减速	
	    TIM_SetCompare2(TIM8,yaw_rightmove_right1);    //头部右侧减速
		}
	}while(-200<pid_yaw.Actualangle<+200);
}

//左翻
void PWM_Leftturn_OUT(void)
{
			static int roll_leftturn_left=100;
			static int roll_leftturn_right=100;

      TIM_SetCompare2(TIM2,roll_leftturn_left);
      TIM_SetCompare3(TIM2,roll_leftturn_right);
}	

//右翻
void PWM_Rightturn_OUT(void)
{
			static int roll_rightturn_left=1500;
			static int roll_rightturn_right=1500;

      TIM_SetCompare2(TIM2,roll_rightturn_left);
      TIM_SetCompare3(TIM2,roll_rightturn_right);
}	

