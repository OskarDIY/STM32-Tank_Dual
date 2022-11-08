/**
  ******************************************************************************
  * @file    power_control.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-06-01
  * @brief   output power control
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
	* 该程序仅供学习使用，未经作者允许，不得用于其它任何用途
  ******************************************************************************
  */

#include "i2c_oled.h"
#include "power_control.h"
#include "key.h"
#include "adc.h"
#include "motor.h"

#include "FreeRTOS.h"
#include "task.h"

 /**
  * @brief  初始化控制可调电源的引脚
  * @param  无
  * @retval 无
  */
void ADJ_Outpt_Init(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	/* 关闭ADJ电源输出	*/
	GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}



 /**
  * @brief  初始化主电源的引脚
  * @param  无
  * @retval 无
  */
void MAIN_Output_Init(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	/* 关闭主电源输出	*/
	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
}


 /**
  * @brief  初始化电源键
  * @param  无
  * @retval 无
  */
void PowerControl_Init(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
//	GPIO_SetBits(GPIOC, GPIO_Pin_0);
}


extern Menu menu;

/* 电源状态用1个字节表示, 0:关闭稳压输出/关闭主电源输出; 1:开启稳压输出/关闭主电源输出;
 * 2:关闭稳压输出/开启主电源输出; 3:开启稳压输出/开启主电源输出
 */
uint8_t powerStatus = 0;


// 电源管理，开关机管理
void PowerTask(void *param)
{
	ADJ_Outpt_Init();
	MAIN_Output_Init();
	PowerControl_Init();
	
	PowerKey_Init();
	
HERE:
	
	// 长按超过1秒开机
	vTaskDelay(1000);
	if(!POWER_KEY_STATUS)
	{
		POWER_ON;
	}
	else
	{
		goto HERE;
//		while(1)
//		{
//			vTaskDelay(100);
//		}
	}
	
	ADJ_OUTPUT_ON;
	MAIN_OUTPUT_ON;
	
	
	// 检测电源键状态
	while(!POWER_KEY_STATUS)      // 开机后电源键未松开
	{
		vTaskDelay(100);
	}
	
	while(menu == START)
	{
		vTaskDelay(100);
	}
	
	while(1)
	{
		if(!POWER_KEY_STATUS)
		{
			vTaskDelay(1500);
			if(!POWER_KEY_STATUS)
			{
				menu = SHUTDOWN;
				vTaskDelay(6000);		// 等待关机动画结束,等待转动中的电机停止,防止电机发电效应导致重新进入开机流程
				while(!POWER_KEY_STATUS);
				POWER_OFF;
			}
		}
		else
		{
			if(powerStatus == 0x00)
			{
				ADJ_OUTPUT_OFF;
				MAIN_OUTPUT_OFF;
			}
			else if(powerStatus == 0x01)
			{
				ADJ_OUTPUT_ON;
				MAIN_OUTPUT_OFF;
			}
			else if(powerStatus == 0x02)
			{
				ADJ_OUTPUT_OFF;
				MAIN_OUTPUT_ON;
			}
			else if(powerStatus == 0x03)
			{
				ADJ_OUTPUT_ON;
				MAIN_OUTPUT_ON;
			}
			
			// 根据之前ADC的采样，计算舵机电源电压
			ADC_ConvertedValueLocal[0] = ADC_ConvertedValueLocal[0] * (0.8) + (float) ADC_ConvertedValue[0]/4096*3.3*11 * (0.2);
			
			// 计算主电源电压
			ADC_ConvertedValueLocal[1] = ADC_ConvertedValueLocal[1] * (0.8) + (float) ADC_ConvertedValue[1]/4096*3.3*20.608 *(0.2);
			vTaskDelay(20);
		}
	}
	
}


/*********************************************END OF FILE**********************/
