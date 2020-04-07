#ifndef __CONTROL_H
#define __CONTROL_H	

void PWM_Forward_OUT(void);	   //前进
void PWM_Back_OUT(void);       //后退
void PWM_Up_OUT(void);         //上升
void PWM_Down_OUT(void);       //下降
void PWM_Left_OUT(void);       //左转
void PWM_Right_OUT(void);      //右转
void PWM_Leftmove_OUT(void);   //左移
void PWM_Rightmove_OUT(void);  //右移
void PWM_Leftturn_OUT(void);   //左翻
void PWM_Rightturn_OUT(void);  //右翻
void ROV_Control(void);        //ROV运动控制函数


#endif

