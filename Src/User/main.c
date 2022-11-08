/**
  ******************************************************************************
  * @file    main.c
  * @author: Oskar Wei
  * @version 2.1
  * @date    2020-05-17
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
	* 该程序仅供学习使用，未经作者允许，不得用于其它任何用途
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include <stdio.h>
#include "led.h"
#include "usart1.h"
#include "uart4.h"
#include "uart5.h"
#include "spi_flash.h"
#include "ff.h"
#include "string.h"
#include "key.h"
#include "i2c_oled.h"
#include "delay.h"
#include "buzzer.h"
#include "watchdog.h"
#include "delay.h"
#include "sensors.h"
#include "utils.h"
#include "power_control.h"
#include "adc.h"
#include "can.h"
#include "servo.h"
#include "motor.h"
#include "radio.h"
#include "flash.h"

#include "FreeRTOS.h"	// 此处"x"提示,是因为Keil误报,与代码无关，没有影响
#include "task.h"

#include "i2c_device.h"

/* Private define ------------------------------------------------------------*/
/* Extern define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




TaskHandle_t startTaskHandle;
extern xTaskHandle radiolinkTaskHandle;

static void startTask(void *arg);

uint8_t can_buf[8] = {'H','i','!','C','A','N','!','\n'};

int main(void)
{
	// 中断优先级分组配置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	// 延时初始化
	Delay_Init(72);
	
	// 蜂鸣器初始化
	Buzzer_Init();
	
	// 用户键初始化
	UserKey_Init();
	
	LED_Init();
	
	USART1_Init(115200);
	printf("Hello  world !\r\n");
	UART4_Init(9600);
	UART5_Init(9600);
	
	i2cdevInit(I2C2_DEV);
	
	// 加载STM32片内Flash存储器中保存的配置数据,包括地址、频率、摇杆校准数据
	LoadConfigData();
	
	FileSystem_Init();
	
	ADCx_Init();
	
	Servo_Init();
	
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_4tq,CAN_BS1_4tq,4,CAN_Mode_Normal);//CAN初始化环回模式,波特率1Mbps
		
	// 初始化看门狗
//	watchdogInit(WATCHDOG_RELOAD_MS);
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 1, &startTaskHandle);	/*创建起始任务*/

	vTaskStartScheduler();	/*开启任务调度*/
	
	
	/* 不再使用文件系统,取消挂载文件系统 */
	f_mount(NULL,"1:",1);
	LED_TOGGLE;
}


// 创建任务
void startTask(void *arg)
{
	// 进入临界区
	taskENTER_CRITICAL();
	
	printf("Free heap before starting: %d bytes\n", xPortGetFreeHeapSize());
	
	xTaskCreate(SensorTask, "SENSORS", 300, NULL, 5, NULL);			        /*创建传感器处理任务*/
	
	xTaskCreate(MenuTask, "MENU", 1600, NULL, 5, NULL);										/*创建显示处理任务*/
	
//	xTaskCreate(PlayMusic, "PlayMusic", 200, NULL, 3, NULL);			      /*创建音乐处理任务*/
	
	xTaskCreate(PowerTask, "POWER", 100, NULL, 5, NULL);
	
	xTaskCreate(MotorTask, "MOTOR", 100, NULL, 5, NULL);
	
	xTaskCreate(RadioTask, "RADIO", 100, NULL, 5, NULL);
	
	xTaskCreate(Usart1Task, "USART1", 200, NULL, 5, NULL);
	
	xTaskCreate(Uart4Task, "UART4", 100, NULL, 5, NULL);
	
	xTaskCreate(Uart5Task, "UART5", 100, NULL, 5, NULL);
	
	// 打印剩余堆栈大小
	printf("Free heap after starting: %d bytes\n", xPortGetFreeHeapSize());
	
	// 删除开始任务
	vTaskDelete(startTaskHandle);
	
	// 退出临界区
	taskEXIT_CRITICAL();
} 




// 空闲任务,CPU空闲时执行
void vApplicationIdleHook( void )
{
//	static u32 tickWatchdogReset = 0;
//	static char pWriteBuffer[2024];

	portTickType tickCount = getSysTickCnt();

//	if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS)
	{
//		tickWatchdogReset = tickCount;
		watchdogReset();
	}
	
//	vTaskList((char *)&pWriteBuffer);
//	printf("task_name  task_state  priority  stack  tasK_num\n");
//	printf("%s\n", pWriteBuffer);
	
	CanSendMsg(can_buf,8);//发送8个字节
	
	__WFI();	/*进入低功耗模式*/
}





void vApplicationMallocFailedHook( void )
{
	portDISABLE_INTERRUPTS();
	printf("\nMalloc failed!\n");
	LED_TOGGLE;
	while(1);
}

#if (configCHECK_FOR_STACK_OVERFLOW == 1)
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
	portDISABLE_INTERRUPTS();
	printf("\nStack overflow!\n");
	LED_TOGGLE;
	while(1);
}
#endif








#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
