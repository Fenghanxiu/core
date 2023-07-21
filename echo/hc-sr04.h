/*
 * hcsr04.h
 *
 *  Created on: Jul 11, 2023
 *      Author: asus
 */
#ifndef _HCSR04_H
#define _HCSR04_H

#include "tim.h"
//配置trig引脚用标签定义

//配置使用的定时器和捕获通道
#define HCSR04_TIM 		 		  TIM2
#define HCSR04_TIM_HAL 			htim2
#define HCSR04_TIM_CHANNEL  TIM_CHANNEL_2
//声明一个状态枚举常量
typedef enum Run_State
{
	TRIG_WAIT = 0,
	RISING,
	FALLING,
	OVER
}Run_State;
//声明结构体方便传递使用数据
typedef struct
{
	Run_State STATE;
	int buf[2];
	float len;
}HCSR04;


//声明一下回调函数
extern void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
//其他函数
void Hcsr04_start(void);
float Hcsr04Read(void);
#endif


