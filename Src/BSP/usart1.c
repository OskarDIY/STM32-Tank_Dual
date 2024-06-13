/**
  ******************************************************************************
  * @file    usart1.c
	* @author  Oskar Wei
  * @version V1.0
  * @date    2021-01-26
  * @brief   调试用的printf串口，重定向printf到串口
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Blog: www.mindsilicon.com
  *
  ******************************************************************************
  */ 

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>

#include "usart1.h"
#include "ff.h"
#include "led.h"
#include "i2c_oled.h"
#include "radio.h"
#include "flash.h"
#include "servo.h"
#include "motor.h"
#include "power_control.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void USART1_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 打开串口GPIO和串口外设的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);  

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

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
	USART_Init(USART1, &USART_InitStructure);
	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure); 

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	

	// 使能串口
	USART_Cmd(USART1, ENABLE);

//#ifdef __GNUC__
//	setvbuf(stdout, NULL, _IONBF, 0);
//#endif
}


// 串口1接收计数
uint16_t usart1RecCount = 0;

// 串口接收队列
xQueueHandle  usart1RxQueue;


// 串口1中断, 接收到的数据是否符合通信协议会在中断服务程序中进行判断, 不符合通信协议的数据将被忽略
void USART1_IRQHandler(void)
{
	// temp用于缓存当前字
	uint8_t temp = 0;
	static radioMsg_t usartMsgTemp;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
	{
		temp = USART_ReceiveData(USART1);
		
		if(usart1RecCount == 0)						// 识别帧头第一个字节
		{
			if(temp == 0xAA)
			{
				usart1RecCount++;
			}
		}
		else if(usart1RecCount == 1)			// 识别帧头第二个字节
		{
			if(temp == 0xAA)
			{
				usart1RecCount++;
				usartMsgTemp.head = 0xAAAA;
			}
			else														// 帧头错误处理
			{
				usart1RecCount = 0;
			}
		}
		else if(usart1RecCount == 2)			// 通信代码
		{
			usart1RecCount++;
			usartMsgTemp.msgID = temp;
		}
		else if(usart1RecCount == 3)			// 数据帧长度
		{
			usart1RecCount++;
			if(temp <= 32)
			{
				usartMsgTemp.dataLen = temp;
			}
			else
			{
				usart1RecCount = 0;
				usartMsgTemp.dataLen = 0;
			}
		}
		else if(usart1RecCount < usartMsgTemp.dataLen - 2)		// 保存数据载荷内容
		{
			usartMsgTemp.data[usart1RecCount - 4] = temp;
			usart1RecCount++;
		}
		else if(usart1RecCount == usartMsgTemp.dataLen - 2)		// 保存校验码第一个字节
		{
			usartMsgTemp.checksum = temp;
			usart1RecCount++;
		}
		else if(usart1RecCount == usartMsgTemp.dataLen - 1)		// 保存校验码第二个字节
		{
			usartMsgTemp.checksum = usartMsgTemp.checksum | temp << 8;
			xQueueSendFromISR(usart1RxQueue, &usartMsgTemp, 0);
			
			usart1RecCount = 0;
			usartMsgTemp.dataLen = 0;
		}
		else
		{
			usart1RecCount = 0;
			usartMsgTemp.dataLen = 0;
		}
		
		
	}
}




static bool usart1LinkStatus = false;

/*获取USART1连接状态*/
bool Usart1LinkStatus(void)
{
	return usart1LinkStatus;
}


// 串口1任务,串口1即USB接口
void Usart1Task(void *param)
{
	usart1RxQueue = xQueueCreate(USART1_RX_QUEUE_SIZE, sizeof(radioMsg_t));
	
	radioMsg_t usart_msg;
	
	uint16_t checkSum = 0;
	
	while(1)
	{
		if(xQueueReceive(usart1RxQueue, &usart_msg, 200) == pdTRUE)
		{
			// 计算CRC16校验值, 与接收到的校验值比较, 判断数据是否发生错误
			checkSum = CRC_Table((uint8_t *)&usart_msg, usart_msg.dataLen - 2);
			
			if(usart_msg.checksum == checkSum)		// 校验通过
			{
				// printf("Received checksum: %x, and calculated checkSum: %x, are mached!\n", usart_msg.checksum, checkSum);
				ProcessMsg(&usart_msg);
				usart1LinkStatus = true;
			}
			else																	// 未通过校验
			{
				printf("USART1 Error: Received checksum: %x, and calculated checkSum: %x, are not mached!\n", usart_msg.checksum, checkSum);
			}
		}
		else
		{
			usart1LinkStatus = false;
		}
	}
}


extern systemConfig_t configParam;
extern motorStatus_t motorStatus;
extern servoPWM_t servoPWM;
extern uint8_t powerStatus;
extern uint8_t ak8963_CaliStage;

extern xQueueHandle  radioTxQueue;


// 消息处理函数, 通信代码可查阅radio.h中的定义
void ProcessMsg(radioMsg_t *msg)
{
	radioMsg_t ackMsg;
	
	// 根据通信代码的不同来做出不同的响应
	if(msg->msgID == CMD_APPTCB1)				// 从上位机发送到控制板的命令1, 读取地址和频率
	{
		ackMsg.head = 0xAAAA;
		ackMsg.msgID = RES_APPTCB1;				// 控制板对APPTCB1的响应, 返回地址和频率
		ackMsg.dataLen = 0x0C;
		
		// 无线地址
		ackMsg.data[0] = configParam.radio.addr[0];
		ackMsg.data[1] = configParam.radio.addr[1];
		ackMsg.data[2] = configParam.radio.addr[2];
		ackMsg.data[3] = configParam.radio.addr[3];
		ackMsg.data[4] = configParam.radio.addr[4];
		
		// 无线频率
		ackMsg.data[5] = configParam.radio.channel;
		
		// CRC16校验和
		ackMsg.checksum = CRC_Table((uint8_t *)&ackMsg, ackMsg.dataLen - 2);
		
		// 发送数据
		for(int i = 0; i < ackMsg.dataLen - 2; i++)
		{
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
			{
				
			}
			USART_SendData(USART1, ((uint8_t *)&ackMsg)[i]);
		}
		
		// 因为STM32单片机是小端模式, 所以校验值的低字节存储在低地址, 先发送校验的低字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum);
		
		// 发送校验的高字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum >> 8);
		
	}
	else if(msg->msgID == CMD_APPTCB2)				// 从上位机发送到控制板的命令2, 写入地址
	{
		configParam.radio.addr[0] = msg->data[0];
		configParam.radio.addr[1] = msg->data[1];
		configParam.radio.addr[2] = msg->data[2];
		configParam.radio.addr[3] = msg->data[3];
		configParam.radio.addr[4] = msg->data[4];
		
		if(SaveConfigData())
		{
			// 写入成功返回1, 写入失败返回0
			ackMsg.data[0] = 0x01;
		}
		else
		{
			ackMsg.data[0] = 0x0;
		}
		
		ackMsg.head = 0xAAAA;
		ackMsg.msgID = RES_APPTCB2;				// 控制板对APPTCB2的响应, 返回写入状态
		ackMsg.dataLen = 0x07;
		
		// CRC16校验和
		ackMsg.checksum = CRC_Table((uint8_t *)&ackMsg, ackMsg.dataLen - 2);
		
		// 发送数据
		for(int i = 0; i < ackMsg.dataLen - 2; i++)
		{
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
			{
				
			}
			USART_SendData(USART1, ((uint8_t *)&ackMsg)[i]);
		}
		
		// 因为STM32单片机是小端模式, 所以校验值的低字节存储在低地址, 先发送校验的低字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum);
		
		// 发送校验的高字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum >> 8);
		
	}
	else if(msg->msgID == CMD_APPTCB3)				// 从上位机发送到控制板的命令3, 写入频率
	{
		configParam.radio.channel = msg->data[0];
		
		if(SaveConfigData())
		{
			// 写入成功返回1, 写入失败返回0
			ackMsg.data[0] = 0x01;
		}
		else
		{
			ackMsg.data[0] = 0x0;
		}
		
		ackMsg.head = 0xAAAA;
		ackMsg.msgID = RES_APPTCB3;				// 控制板对APPTCB3的响应, 返回写入状态
		ackMsg.dataLen = 0x07;
		
		// CRC16校验和
		ackMsg.checksum = CRC_Table((uint8_t *)&ackMsg, ackMsg.dataLen - 2);
		
		// 发送数据
		for(int i = 0; i < ackMsg.dataLen - 2; i++)
		{
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
			{
				
			}
			USART_SendData(USART1, ((uint8_t *)&ackMsg)[i]);
		}
		
		// 因为STM32单片机是小端模式, 所以校验值的低字节存储在低地址, 先发送校验的低字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum);
		
		// 发送校验的高字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum >> 8);
		
	}
	else if(msg->msgID == CMD_APPTCB5) // 从上位机发送到控制板的命令5, 模拟无线手柄操作, 上位机按照手柄的格式发送数据给小车
	{
		ParseRadioMsg(msg);
	}
	else if(msg->msgID == CMD_APPTCB6) // 从上位机发送到接收机的命令6,设置8路舵机PWM值和电机转速,电机转速弧度每秒,开启2路电源对外输出
	{
		// 命令格式示例:AA AA 16 1B 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 5E A6
		// 设置舵机
		servoPWM.servo1 = (msg->data[1] << 8) | msg->data[0];
		servoPWM.servo2 = (msg->data[3] << 8) | msg->data[2];
		servoPWM.servo3 = (msg->data[5] << 8) | msg->data[4];
		servoPWM.servo4 = (msg->data[7] << 8) | msg->data[6];
		servoPWM.servo5 = (msg->data[9] << 8) | msg->data[8];
		servoPWM.servo6 = (msg->data[11] << 8) | msg->data[10];
		servoPWM.servo7 = (msg->data[13] << 8) | msg->data[12];
		servoPWM.servo8 = (msg->data[15] << 8) | msg->data[14];
		
		// 设置电机
		motorStatus.target_speed1 = (msg->data[16] << 8) | msg->data[17];
		motorStatus.target_speed2 = (msg->data[18] << 8) | msg->data[19];
		
		// 设置电源对外输出
		powerStatus = msg->data[20];
		
		// 返回数据格式示例:AA AA 1E 11 00 00 00 00 00 00 00 00 00 00 00 7F F6
		ackMsg.head = 0xAAAA;
		ackMsg.msgID = RES_APPTCB6;				// 控制板对APPTCB3的响应, 返回写入状态
		ackMsg.dataLen = 0x11;
		
		ackMsg.data[0] = 0;
		
		ackMsg.data[1] = motorStatus.current_speed1 >> 8;
		ackMsg.data[2] = motorStatus.current_speed1;
		
		ackMsg.data[3] = motorStatus.current_speed2 >> 8;
		ackMsg.data[4] = motorStatus.current_speed2;

//		ackMsg.data[1] = motorStatus.motor1_pulse >> 8;
//		ackMsg.data[2] = motorStatus.motor1_pulse;
//		
//		ackMsg.data[3] = motorStatus.motor2_pulse >> 8;
//		ackMsg.data[4] = motorStatus.motor2_pulse;
		
		// CRC16校验和
		ackMsg.checksum = CRC_Table((uint8_t *)&ackMsg, ackMsg.dataLen - 2);
		
		// 发送数据
		for(int i = 0; i < ackMsg.dataLen - 2; i++)
		{
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
			{

			}
			USART_SendData(USART1, ((uint8_t *)&ackMsg)[i]);
		}

		// 因为STM32单片机是小端模式, 所以校验值的低字节存储在低地址, 先发送校验的低字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum);

		// 发送校验的高字节
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
		{

		}
		USART_SendData(USART1, ackMsg.checksum >> 8);
	}
	else if(msg->msgID == CMD_APPTCB9)	// 从上位机发送到手柄的命令,校准主板磁力计传感器,通知下位机发送原始数据, 发送校准结果给下位机
	{
		// 会有3个处理阶段,
		// 第一阶段收到上位机读取磁力计数据的指令, 然后持续向上位机发送磁力计数据
		// 第二个阶段, 上位机已经收集到了足够的数据, 发送指令要求下位机停止发送数据
		// 第三个阶段, 上位机已经计算好了校准参数, 这是上位机发送校准参数给下位机, 下位机保存此数据
		// 我们通过状态代码来判断当前校准的阶段, 1:第一阶段, 2:第二阶段, 3:第三阶段
		// 命令格式示例:AA AA 21 len status _ _
		if(msg->data[0] ==  1)
		{
			ak8963_CaliStage = 1;
		}
		else if(msg->data[0] ==  2)
		{
			ak8963_CaliStage = 2;
		}
		else if(msg->data[0] == 3)
		{
			ak8963_CaliStage = 3;
		}
		else
		{
			ak8963_CaliStage = 0;
		}
		
	}
	else if((msg->msgID == RES_RTCB1) || (msg->msgID == RES_APPTCB5)) // 回传数据给手柄, 透传功能, 接收机发送数据给上位机
	{
		xQueueSend(radioTxQueue, msg, 0);
	}
	
}



#ifdef __GNUC__


#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

int _isatty(int fd)
{
	if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
		return 1;

	errno = EBADF;
	return 0;
}

int _getpid()
{
	return 0;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}


int _write(int fd, char *ptr, int len)
{
	for(uint16_t t=0; t<len; t++)
	{
		//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		USART_SendData(USART1, ptr[t]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}
	return len;
}

int _read(int fd, char* ptr, int len)
{
	for(int t=0; t<len; t++)
	{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		ptr[t] = (char)USART_ReceiveData(USART1);
	}
	return len;
}


int _close(int fd)
{
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}


int _lseek(int fd, int ptr, int dir)
{
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st)
{
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
  {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}



#elif

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART1, (uint8_t) ch);

		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (char)USART_ReceiveData(USART1);
}

#endif






