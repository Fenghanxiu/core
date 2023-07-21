/*
 * hcsr04.c
 *
 *  Created on: Jul 11, 2023
 *      Author: asus
 */
#include "hcsr04.h"
//声明一个模块的数据结构体
HCSR04 hcsr04;
//重新编写输入捕获回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(HCSR04_TIM == htim->Instance)
	{
		//如果上升沿检测
		if(hcsr04.STATE == RISING)
		{
			//设置定时器CNT为0
			__HAL_TIM_SET_COUNTER(&HCSR04_TIM_HAL,0);
		  //读取上升沿时的CNT值到buf
			hcsr04.buf[0] = __HAL_TIM_GetCounter(&HCSR04_TIM_HAL);
			//设置下一个捕获为下降沿
			__HAL_TIM_SET_CAPTUREPOLARITY(&HCSR04_TIM_HAL,HCSR04_TIM_CHANNEL,TIM_ICPOLARITY_FALLING);
			//改变运行模式
			hcsr04.STATE=FALLING;
		}else if(hcsr04.STATE == FALLING)//如果下降沿检测
		{
			//获取当前的CNT到buf2，这样高电平维持的时间长度就记录了
			hcsr04.buf[1] = __HAL_TIM_GetCounter(&HCSR04_TIM_HAL);
			//将运行标志设置为完成
			hcsr04.STATE=OVER;
		}
	}

}

void Hcsr04_start(void)
{
	if(hcsr04.STATE == TRIG_WAIT)
		  		{
		  			//给HCSR04一个1ms触发电平
		  			HAL_GPIO_WritePin(GPIOA,Trig_Pin,1);
		  			HAL_Delay(1);
		  			HAL_GPIO_WritePin(GPIOA,Trig_Pin,0);
		  			//启动上升沿捕获
		  			__HAL_TIM_SET_CAPTUREPOLARITY(&HCSR04_TIM_HAL, HCSR04_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
		  			//启动输入捕获
		  			HAL_TIM_IC_Start_IT(&HCSR04_TIM_HAL, HCSR04_TIM_CHANNEL);
		  			//设置上升沿捕获
		  			hcsr04.STATE = RISING;
		  		}

		  		if(hcsr04.STATE == OVER)//判断电平捕获结束
		  		{
		  			//计算高电平时间差值代表时间维持的长度，因为定时器设置1us自加一次，所以时长单位为us
		  			//乘以0.017的是因为声波速度为340m/s，而1s=1000000us，且技术一次距离是一个来回
		  			//要除以2，此处单位为cm
		  			hcsr04.len = (float)(hcsr04.buf[1]- hcsr04.buf[0])*0.017;
		  			//刷新状态，使可以进行下一次触发电平
		  			hcsr04.STATE = TRIG_WAIT;
		  			HAL_TIM_IC_Stop_IT(&HCSR04_TIM_HAL, HCSR04_TIM_CHANNEL);
		  			//发送数据到上位机
		  			//printf("HCSR04_Len: %.3f cm \r\n",hcsr04.len);
		  			//延时1s
		  			//HAL_Delay(1000);//可加延迟
		  		}

}

float Hcsr04Read(void)
{
	// 测距结果限幅
	  if(hcsr04.len >= 450)
	  {
		  hcsr04.len = 450;
	  }
	  return hcsr04.len;
}


