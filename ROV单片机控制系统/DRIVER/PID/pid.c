#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "pid.h"

struct _pid pid_roll,pid_pitch,pid_yaw;

//pid初始函数
void PID_init(void)
{
    pid_roll.Setangle=0.0;
    pid_roll.Actualangle=0.0;
    pid_roll.err=0.0;
    pid_roll.err_last=0.0;
    pid_roll.control=0.0;
    pid_roll.integral=0.0;
    pid_roll.Kp=0.0;
    pid_roll.Ki=0.0;       //稍微增大积分环节的值
    pid_roll.Kd=0.0;
    pid_roll.umax=200;
    pid_roll.umin=-200;
	
	  pid_pitch.Setangle=0.0;
    pid_pitch.Actualangle=0.0;
    pid_pitch.err=0.0;
    pid_pitch.err_last=0.0;
    pid_pitch.control=0.0;
    pid_pitch.integral=0.0;
    pid_pitch.Kp=0.0;
    pid_pitch.Ki=0.0;       //稍微增大积分环节的值
    pid_pitch.Kd=0.0;
    pid_pitch.umax=200;
    pid_pitch.umin=-200;

	  pid_yaw.Setangle=0.0;
    pid_yaw.Actualangle=0.0;
    pid_yaw.err=0.0;
    pid_yaw.err_last=0.0;
    pid_yaw.control=0.0;
    pid_yaw.integral=0.0;
    pid_yaw.Kp=0.0;
    pid_yaw.Ki=0.0;       //稍微增大积分环节的值
    pid_yaw.Kd=0.0;
    pid_yaw.umax=200;
    pid_yaw.umin=-200;
}

//Kp比例系数的设置
void PIDSetKp(float Kp1,float Kp2,float Kp3)
{
	pid_roll.Kp=Kp1;
	pid_pitch.Kp=Kp2;
	pid_yaw.Kp=Kp3;
}

//Ki积分系数的设置
void PIDSetKi(float Ki1,float Ki2,float Ki3)
{
	pid_roll.Ki=Ki1;
	pid_pitch.Ki=Ki2;
	pid_yaw.Ki=Ki3;
}

//Kd微分系数的设置
void PIDSetKd(float Kd1,float Kd2,float Kd3)
{
	pid_roll.Kd=Kd1;
	pid_pitch.Kd=Kd2;
	pid_yaw.Kd=Kd3;
}

//pid--roll角 实现函数
float PID_realize_roll(float angle_roll)
{
    int index;
    pid_roll.Setangle=angle_roll;
    pid_roll.err=pid_roll.Setangle-pid_roll.Actualangle;   //偏差量

   if(pid_roll.Actualangle>pid_roll.umax)  //抗积分饱和的实现
    {
       if(abs(pid_roll.err)>angle_roll)      //积分分离
        {
            index=0;
        }else{
            index=1;
            if(pid_roll.err<0)
            {
               pid_roll.integral+=pid_roll.err;
            }
        }
    }else if(pid_roll.Actualangle<pid_roll.umin){
        if(abs(pid_roll.err)>angle_roll)      //积分分离
        {
            index=0;
        }else{
            index=1;
            if(pid_roll.err>0)
            {
             pid_roll.integral+=pid_roll.err;
            }
        }
    }else{
        if(abs(pid_roll.err)>angle_roll)       //积分分离
        {
            index=0;
        }else{
            index=1;
             pid_roll.integral+=pid_roll.err;
        }
    }
    pid_roll.control=pid_roll.Kp*pid_roll.err+index*pid_roll.Ki*pid_roll.integral+pid_roll.Kd*(pid_roll.err-pid_roll.err_last);
    pid_roll.err_last=pid_roll.err;
    pid_roll.control=pid_roll.control*1.0;
    return pid_roll.control;
}


//pid--pitch角 实现函数
float PID_realize_pitch(float angle_pitch)
{
    int index;
    pid_pitch.Setangle=angle_pitch;
    pid_pitch.err=pid_pitch.Setangle-pid_pitch.Actualangle;   //偏差量

   if(pid_pitch.Actualangle>pid_pitch.umax)  //抗积分饱和的实现
    {
       if(abs(pid_pitch.err)>angle_pitch)      //积分分离
        {
            index=0;
        }else{
            index=1;
            if(pid_pitch.err<0)
            {
              pid_pitch.integral+=pid_pitch.err;
            }
        }
    }else if(pid_pitch.Actualangle<pid_pitch.umin){
        if(abs(pid_pitch.err)>angle_pitch)      //积分分离
        {
            index=0;
        }else{
            index=1;
            if(pid_pitch.err>0)
            {
            pid_pitch.integral+=pid_pitch.err;
            }
        }
    }else{
        if(abs(pid_pitch.err)>angle_pitch)       //积分分离
        {
            index=0;
        }else{
            index=1;
            pid_pitch.integral+=pid_pitch.err;
        }
    }
    pid_pitch.control=pid_pitch.Kp*pid_pitch.err+index*pid_pitch.Ki*pid_pitch.integral+pid_pitch.Kd*(pid_pitch.err-pid_pitch.err_last);
    pid_pitch.err_last=pid_pitch.err;
    pid_pitch.control=pid_pitch.control*1.0;
    return pid_pitch.control;
}


//pid--yaw角 实现函数
float PID_realize_yaw(float angle_yaw)
{
    int index;
    pid_yaw.Setangle=angle_yaw;
    pid_yaw.err=pid_yaw.Setangle-pid_yaw.Actualangle;   //偏差量

   if(pid_yaw.Actualangle>pid_yaw.umax)  //抗积分饱和的实现
    {
       if(abs(pid_yaw.err)>angle_yaw)      //积分分离
        {
            index=0;
        }else{
            index=1;
            if(pid_yaw.err<0)
            {
              pid_yaw.integral+=pid_yaw.err;
            }
        }
    }else if(pid_yaw.Actualangle<pid_yaw.umin){
        if(abs(pid_yaw.err)>angle_yaw)      //积分分离
        {
            index=0;
        }else{
            index=1;
            if(pid_yaw.err>0)
            {
            pid_yaw.integral+=pid_yaw.err;
            }
        }
    }else{
        if(abs(pid_yaw.err)>angle_yaw)       //积分分离
        {
            index=0;
        }else{
            index=1;
            pid_yaw.integral+=pid_yaw.err;
        }
    }
    pid_yaw.control=pid_yaw.Kp*pid_yaw.err+index*pid_yaw.Ki*pid_yaw.integral+pid_yaw.Kd*(pid_yaw.err-pid_yaw.err_last);
    pid_yaw.err_last=pid_yaw.err;
    pid_yaw.control=pid_yaw.control*1.0;
    return pid_yaw.control;
}



//限制幅度函数
float Math_fConstrain(float value, float min, float max)
{
if(value > max)value = max;
	else if(value < min)value = min;
return value;
}


//姿态角PID控制函数
void roll_pitch_yaw_anglePID(float angle_roll,float angle_pitch,float angle_yaw)
{
	float roll,pitch,yaw;
	//ROLL
	roll=PID_realize_roll(angle_roll);	 //目标角度
  pid_roll.control= Math_fConstrain(roll,-200,+200);  //限制控制PWM信号的幅度
	//PITCH
	pitch=PID_realize_pitch(angle_pitch);	 //目标角度
  pid_pitch.control= Math_fConstrain(pitch,-200,+200);  //限制控制PWM信号的幅度
	//YAW
	yaw=PID_realize_yaw(angle_yaw);	 //目标角度
  pid_yaw.control= Math_fConstrain(yaw,-200,+200);  //限制控制PWM信号的幅度
}

