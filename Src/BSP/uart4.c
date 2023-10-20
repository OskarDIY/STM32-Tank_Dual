/**
  ******************************************************************************
  * @file    uart4.c
	* @author  Oskar Wei
  * @version V1.0
  * @date    2021-01-26
  * @brief   
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Blog: www.mindsilicon.com
  *
  ******************************************************************************
  */ 

#include "usart1.h"
#include "uart4.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "led.h"
#include "i2c_oled.h"
#include "radio.h"
#include "flash.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



 /**
  * @brief  UART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void UART4_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 打开串口GPIO和串口外设的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = baudrate;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(UART4, &USART_InitStructure);
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;          // SCLK时钟配置(同步模式下需要)
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;                // 时钟极性(同步模式下需要)
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;              // 时钟相位(同步模式下需要)
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;      // 最后一位时钟脉冲(同步模式下需要)
	USART_ClockInit(UART4, &USART_ClockInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure); 

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	// 使能串口
	USART_Cmd(UART4, ENABLE);
	
//	USART_SendData(UART4, 0x3A);
}


// 串口4接收计数
uint16_t uart4RecCount = 0;

// 串口4接收队列
xQueueHandle uart4RxQueue;

// 串口4中断, 接收到的数据是否符合通信协议会在中断服务程序中进行判断, 不符合通信协议的数据将被忽略
//void UART4_IRQHandler(void)
//{
//	// temp用于缓存当前字
//	uint8_t temp = 0;
//	static radioMsg_t usartMsgTemp;
//	
//	if(USART_GetITStatus(UART4, USART_IT_RXNE)!=RESET)
//	{
//		temp = USART_ReceiveData(UART4);
//		
//		if(uart4RecCount == 0)						// 识别帧头第一个字节
//		{
//			if(temp == 0xAA)
//			{
//				uart4RecCount++;
//			}
//		}
//		else if(uart4RecCount == 1)			// 识别帧头第二个字节
//		{
//			if(temp == 0xAA)
//			{
//				uart4RecCount++;
//				usartMsgTemp.head = 0xAAAA;
//			}
//			else														// 帧头错误处理
//			{
//				uart4RecCount = 0;
//			}
//		}
//		else if(uart4RecCount == 2)			// 通信代码
//		{
//			uart4RecCount++;
//			usartMsgTemp.msgID = temp;
//		}
//		else if(uart4RecCount == 3)			// 数据帧长度
//		{
//			uart4RecCount++;
//			if(temp <= 32)
//			{
//				usartMsgTemp.dataLen = temp;
//			}
//			else
//			{
//				uart4RecCount = 0;
//				usartMsgTemp.dataLen = 0;
//			}
//		}
//		else if(uart4RecCount < usartMsgTemp.dataLen - 2)		// 保存数据载荷内容
//		{
//			usartMsgTemp.data[uart4RecCount - 4] = temp;
//			uart4RecCount++;
//		}
//		else if(uart4RecCount == usartMsgTemp.dataLen - 2)		// 保存校验码第一个字节
//		{
//			usartMsgTemp.checksum = temp;
//			uart4RecCount++;
//		}
//		else if(uart4RecCount == usartMsgTemp.dataLen - 1)		// 保存校验码第二个字节
//		{
//			usartMsgTemp.checksum = usartMsgTemp.checksum | temp << 8;
//			xQueueSendFromISR(uart4RxQueue, &usartMsgTemp, 0);
//			
//			uart4RecCount = 0;
//			usartMsgTemp.dataLen = 0;
//		}
//		else
//		{
//			uart4RecCount = 0;
//			usartMsgTemp.dataLen = 0;
//		}
//		
//		
//	}
//}


//void Uart4Task(void *param)
//{
//	uart4RxQueue = xQueueCreate(UART4_RX_QUEUE_SIZE, sizeof(radioMsg_t));
//	
//	radioMsg_t usart_msg;
//	
//	uint16_t checkSum = 0;
//	
//	while(1)
//	{
//		if(xQueueReceive(uart4RxQueue, &usart_msg, 10) == pdTRUE)
//		{
//			// 计算CRC16校验值, 与接收到的校验值比较, 判断数据是否发生错误
//			checkSum = CRC_Table((uint8_t *)&usart_msg, usart_msg.dataLen - 2);
//			
//			if(usart_msg.checksum == checkSum)		// 校验通过
//			{
////				printf("Received checksum: %x, and calculated checkSum: %x, are mached!\n", usart_msg.checksum, checkSum);
////				ProcessMsg(&usart_msg);
//			}
//			else																	// 未通过校验
//			{
//				printf("UART4 Error: Received checksum: %x, and calculated checkSum: %x, are not mached!\n", usart_msg.checksum, checkSum);
//			}
//		}
//	}
//}



// 用于保存超声波距离,单位毫米
uint16_t ultrasonic_distance1 = 0;

// 用于保存测量状态, 0:首字节, 1:尾字节, 2:测量完成
uint8_t ultrasonic_state1 = 0;


void Uart4Task(void *param)
{
	vTaskDelay(3000);
	
	
	// 向US100超声波模块发送测量指令0x55
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE)== RESET)
	{

	}
	USART_SendData(UART4, 0x55);
	ultrasonic_state1 = 0;
	
	while(1)
	{
		if(ultrasonic_state1 == 2)
		{
			printf("ultrasonic_distance: %d mm\n", ultrasonic_distance1);
			
			// 向US100超声波模块发送测量指令0x55
			while(USART_GetFlagStatus(UART4, USART_FLAG_TXE)== RESET)
			{

			}
			USART_SendData(UART4, 0x55);
			ultrasonic_state1 = 0;
		}
		
		// 每秒测量10次
		vTaskDelay(100);
	}
}



// 串口4中断,
void UART4_IRQHandler(void)
{
	static uint16_t temp = 0;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE)!=RESET)
	{
		if(ultrasonic_state1 == 0)
		{
			temp = USART_ReceiveData(UART4) << 8;
			ultrasonic_state1 ++;
		}
		else if(ultrasonic_state1 == 1)
		{
			temp |= USART_ReceiveData(UART4);
			ultrasonic_distance1 = temp;
			ultrasonic_state1 ++;
		}
		else
		{
			USART_ReceiveData(UART4);
		}
		
	}
}

