/**
  ******************************************************************************
  * @file    key.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-05-02
  * @brief   key low level init
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
  ******************************************************************************
  */
  
#include "key.h"   

 /**
  * @brief  初始化用户键
  * @param  无
  * @retval 无
  */
void UserKey_Init(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   

	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}


 /**
  * @brief  初始化电源键
  * @param  无
  * @retval 无
  */
void PowerKey_Init(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	
	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}



/*********************************************END OF FILE**********************/
