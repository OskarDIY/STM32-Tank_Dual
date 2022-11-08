/**
  ******************************************************************************
  * @file    buzzer.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-05-02
  * @brief   buzzer low level init
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


//void TIM6_Init(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	// 开启定时器时钟,即内部时钟CK_INT = 72M
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

//	/*--------------------时基结构体初始化-------------------------*/
//	// 配置周期，这里配置为1K
//	
//	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
//	TIM_TimeBaseStructure.TIM_Period = 65535;	
//	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
//	TIM_TimeBaseStructure.TIM_Prescaler = 7199;	
//	// 时钟分频因子 ，配置死区时间时需要用到
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		
//	// 计数器计数模式，设置为向上计数
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		
//	// 重复计数器的值，没用到不用管
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
//	// 初始化定时器
//	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
//	//NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	TIM_ClearFlag(TIM6, TIM_FLAG_Update);//清除TIM的更新标志位
//	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
//	
//	//Reset counter
//  TIM_SetCounter(TIM6, 0);
//	// 使能计数器
//	TIM_Cmd(TIM6, DISABLE);
//}


/* ----------------   PWM信号 周期和占空比的计算--------------- */
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1)
void Buzzer_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	// 输出比较通道1 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	// 开启定时器时钟,即内部时钟CK_INT = 72M
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为1K
	
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period = 999;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	
	// 时钟分频因子 ，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/*--------------------输出比较结构体初始化-------------------*/	
	
	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	// 输出通道空闲电平极性配置
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	
	// 输出比较通道 1
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	
	
	// 主输出使能，当使用的是通用定时器时，这句不需要
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	
	// 使能计数器
	TIM_Cmd(TIM1, DISABLE);
	
	
//	TIM6_Init();
}


// 音调频率表
const uint16_t tone[] = {247,262,294,330,349,392,440,294,523,587,659,698,784,1000};


// 红尘情歌,音调表
const uint8_t music[] = {5,5,6,8,7,6,5,6,13,13,
									5,5,6,8,7,6,5,3,13,13,
									2,2,3,5,3,5,6,3,2,1,
									6,6,5,6,5,3,6,5,13,13,

									5,5,6,8,7,6,5,6,13,13,
									5,5,6,8,7,6,5,3,13,13,
									2,2,3,5,3,5,6,3,2,1,
									6,6,5,6,5,3,6,1,	

									13,8,9,10,10,9,8,10,9,8,6,
									13,6,8,9,9,8,6,9,8,6,5,
									13,2,3,5,5,3,5,5,6,8,7,6,
									6,10,9,9,8,6,5,6,8};

// 红尘情歌,节拍表
const uint8_t time[] = {2,4,2,2,2,2,2,8,4, 4,
									2,4,2,2,2,2,2,8,4, 4, 
									2,4,2,4,2,2,4,2,2,8,
									2,4,2,2,2,2,2,8,4 ,4, 

									2,4,2,2,2,2,2,8,4, 4, 
									2,4,2,2,2,2,2,8,4, 4, 
									2,4,2,4,2,2,4,2,2,8,
									2,4,2,2,2,2,2,8,

									4, 2,2,2, 4, 2,2,2, 2,2,8,
									4, 2,2,2,4,2,2,2,2,2,8,
									4, 2,2,2,4,2,2,5,2,6,2,4,
									2,2 ,2,4,2,4,2,2,12};	

int music_len = sizeof(music)/sizeof(music[0]);
int music_index = 0;
									
void PlayMusic(void *param)
{
	music_index = 0;
	
	TIM_SetCounter(TIM1, 0);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	while(1)
	{
//		printf(">>PlayMusic Task...\r\n");
		
		if(music_index < music_len)
		{
			TIM1->ARR = 1000000 / tone[music[music_index]];
			TIM1->CCR1 = tone[music[music_index]];
			
			TIM_SetCounter(TIM1, 0);
			TIM_Cmd(TIM1, ENABLE);
			TIM_CtrlPWMOutputs(TIM1, ENABLE);
			
			delay_ms(100 * time[music_index]);
			
			TIM_SetCounter(TIM1, 0);
			TIM_Cmd(TIM1, DISABLE);
			TIM_CtrlPWMOutputs(TIM1, DISABLE);
			
			music_index++;
		}
		else
		{
			TIM_SetCounter(TIM1, 0);
			TIM_Cmd(TIM1, DISABLE);
			TIM_CtrlPWMOutputs(TIM1, DISABLE);
			
			delay_ms(1000);
			
			music_index = 0;
		}
	}
	
}

