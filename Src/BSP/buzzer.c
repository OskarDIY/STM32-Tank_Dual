/**
  ******************************************************************************
  * @file    buzzer.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-05-02
  * @brief   buzzer low level init 蜂鸣器驱动程序
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
  ******************************************************************************
  */
  
#include "buzzer.h"   
#include "delay.h"
#include "led.h"
#include <stdio.h>


/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"



void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// 开启定时器时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为1K
	
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period = 30;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;	
	// 时钟分频因子 ，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	
	//Reset counter
  TIM_SetCounter(TIM6, 0);
	TIM_Cmd(TIM6, DISABLE);
}


void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update); // 清除中断标志位
		TOGGLE_BUZZER;
	}
}




// 初始化蜂鸣器
void Buzzer_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	BUZZER_OFF;
	
	TIM6_Init();
}

// 当前音乐类型
uint8_t currMusic = MUSIC_MUTE;
									
void PlayMusic(void *param)
{
	// 蜂鸣器初始化
	Buzzer_Init();
	//TIM_Cmd(TIM6, ENABLE);
	
	while(1)
	{
//		printf(">>PlayMusic Task...\r\n");
		if(currMusic == MUSIC_MUTE)
		{
			vTaskDelay(20);
		}
		else
		{
			TIM_Cmd(TIM6, ENABLE);
			vTaskDelay(100);
			TIM_Cmd(TIM6, DISABLE);
			vTaskDelay(100);
			
			currMusic = MUSIC_MUTE;
			vTaskDelay(50);
		}
		
		
	}
	
}

