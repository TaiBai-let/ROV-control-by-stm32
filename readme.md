## This is project to control ROV by STM32F103***
（Of course, also support all stm32 series of SCM.
But maybe IIC and DMA are used differently）

Design ideas:
      STM32 output PWM to control motor motor  by electric regulation
      and  Mpu6050 is used to obtain the attitude
      use PID algorithm to adjust ROV driving
      And there are controls for the robot arm and the camera cradle
      Get the command from the host computer.

下载代码前也请先联系我，这不是最新版本的。
    直接使用有可能会使单片机受损
如果您有建议和疑惑，可以联系我
QQ：1756170618