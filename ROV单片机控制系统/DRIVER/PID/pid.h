#ifndef __PID_H
#define __PID_H	

//定义变量
struct _pid{
    float Setangle;            //设置角度
    float Actualangle;         //实际角度
    float err;                 //偏差值
    float err_last;            //上一个偏差值
    float Kp,Ki,Kd;            //比例，积分，微分系数
    float control;             //控制执行器的变量
    float integral;            //积分值
    float umax;                //控制最大值
    float umin;                //控制最小值
};

void PID_init(void);     //初始化函数
void PIDSetKp(float Kp1,float Kp2,float Kp3);    //设置比例系数
void PIDSetKi(float Ki1,float Ki2,float Ki3);    //设置积分系数
void PIDSetKd(float Kd1,float Kd2,float Kd3);    //设置微分系数
float PID_realize_roll(float angle_roll);        //实现roll角度的PID控制
float PID_realize_pitch(float angle_pitch);      //实现pitch角度的PID控制
float PID_realize_yaw(float angle_yaw);          //实现yaw角度的PID控制
float Math_fConstrain(float value, float min, float max);   //限制控制PWM信号的幅度
void roll_pitch_yaw_anglePID(float angle_roll,float angle_pitch,float angle_yaw);  //姿态角PID控制函数

#endif

