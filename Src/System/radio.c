/**
  ******************************************************************************
  * @file    radio.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-06-09
  * @brief   radio api
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
	* 该程序仅供学习使用，未经作者允许，不得用于其它任何用途
  ******************************************************************************
  */

#include "obstacle.h"
#include "motor.h"
#include "led.h"
#include "flash.h"
#include "nrf24l01.h"
#include "radio.h" 
#include "i2c_oled.h"
#include "usart1.h"
#include "servo.h"
#include <stdio.h>
#include <string.h>

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"





/*接收队列信息个数*/
#define  RADIOLINK_TX_QUEUE_SIZE  3

xTaskHandle radiolinkTaskHandle;
xQueueHandle  radioTxQueue;
//xQueueHandle  radioRxQueue;
static xSemaphoreHandle nrfIT;

static bool radioConnectStatus = false;
static uint16_t failReceiveCount = 0;

// 系统设置
systemConfig_t configParam;



systemConfig_t configParamDefault=
{
	.version = VERSION,
	.radio.channel = RADIO_CHANNEL,
	.radio.dataRate = RADIO_DATARATE,
	.radio.addr = RADIO_ADDRESS,
	.fs_mark = 0xFFFFFFFF,
};

/*nrf外部中断回调函数*/
static void nrf_interruptCallback(void)
{
	portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(nrfIT, &xHigherPriorityTaskWoken);
}

/*无线配置初始化（地址、通道、速率）*/
static void radioInit(void)
{
	// 加载STM32片内Flash存储器中保存的配置数据,包括地址、频率、摇杆校准数据
	LoadConfigData();
	
	
	if(nrf_check() == SUCCESS)
	{
		NRF_Init(PRX_MODE);
		nrf_setIterruptCallback(nrf_interruptCallback);
	}
	else
	{
		printf("Radio check fail!/n");
		while(1);
	}
}


/*无线连接初始化*/
void radiolinkInit(void)
{
	radioInit();
	
	radioTxQueue = xQueueCreate(RADIOLINK_TX_QUEUE_SIZE, sizeof(radioMsg_t));
	ASSERT(radioTxQueue);
	
//	radioRxQueue = xQueueCreate(RADIOLINK_RX_QUEUE_SIZE, sizeof(radioMsg_t));
//	ASSERT(radioRxQueue);
	
	nrfIT = xSemaphoreCreateBinary();
}



/*无线连接任务*/
void RadioTask(void* param)
{
	radioMsg_t rx_p;
	uint16_t checkSum;
	uint8_t temp[sizeof(radioMsg_t)];

	radiolinkInit();	/*无线通信初始化*/
	
	// RX active
	NRF_CE_H();
	
	while(1)
	{
//		printf("Radio Task\n");
		LED_TOGGLE;
		
		if(xQueueReceive(radioTxQueue, &rx_p, 0) == pdTRUE)
		{
			memcpy(temp, (uint8_t *)&rx_p, rx_p.dataLen - 2);
			temp[rx_p.dataLen - 2] = rx_p.checksum;
			temp[rx_p.dataLen - 1] = rx_p.checksum >> 8;
			nrf_txPacket_AP(temp, rx_p.dataLen);
		}
		
		xSemaphoreTake(nrfIT, 100);
		
		nrfEvent_e status = nrf_checkEventandRxPacket((uint8_t *)&rx_p, &(rx_p.dataLen));
		rx_p.checksum = ((uint8_t *)&rx_p)[rx_p.dataLen - 2] | ((uint8_t *)&rx_p)[rx_p.dataLen - 1] << 8;
		
		// 如果未收到数据
		if(status == IDLE)
		{
			failReceiveCount ++;
			if(failReceiveCount > 2)
			{
				radioConnectStatus = false;
			}
			continue;
		}
		
		checkSum = CRC_Table((uint8_t *)&rx_p, rx_p.dataLen - 2);
		if((rx_p.dataLen <= RADIO_MSG_MAX_SIZE) && ((rx_p.checksum & 0xFF) == (checkSum & 0xFF)) && ((rx_p.checksum >> 8) == (checkSum >> 8)))
		{
			ParseRadioMsg(&rx_p);
			failReceiveCount = 0;
			radioConnectStatus = true;
		}
		else
		{
			failReceiveCount ++;
			if(failReceiveCount > 2)
			{
				radioConnectStatus = false;
			}
			continue;
		}
		
	}
}


extern motorStatus_t motorStatus;
extern servoPWM_t servoPWM;

// 处理无线数据
void ParseRadioMsg(radioMsg_t *msg)
{
	int16_t pwm1 = 0, pwm2 = 0;
	static float duty[8] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
	uint16_t keys = 0;
	
	// 先计算电机的PWM值, data[4]是右侧摇杆垂直方向, 用来控制前后运动
	pwm1 = -(msg->data[4] - 0x7F) * 8;
	pwm2 = pwm1;
	
	// data[3]是右侧摇杆水平方向, 用来控制左右转向时电机的差速
	pwm1 += (msg->data[3] - 0x7F) * 5;
	pwm2 -= (msg->data[3] - 0x7F) * 5;
	
	//ObstacleAvoidance(&pwm1, &pwm2);
	
	motorStatus.motor1_pwm = pwm1;
	motorStatus.motor2_pwm = pwm2;
	
	// 1号舵机控制, data[1]是左侧摇杆水平方向, 用来控制转向舵机
	duty[0] = -(float)(msg->data[1] - 0x7F) / 0x7F;
	// 除以6是控制舵机转动幅度, 加上0.5使摇杆回中
	duty[0] = duty[0] / 7 + 0.5;
	// 转向舵机插在1号舵机接口
	PWM(&duty[0]);
	servoPWM.servo1 = Duty_to_PWM(duty[0]);
	
	// 其他舵机的控制, 用按键控制, 分别为data[5]和data[6]
	// data的第6和第7字节, 这两个字节共16位, 对应手柄的16个通道, 
	// 从最高位第16位到第1位, 依次是：L2, L1, LU, LL, LD, LR, SE, ST, RL, RD, RR, RU, R1, R2, R-KEY, L-KEY
	// 其中R-KEY和L-KEY分别是左右摇杆向下按下对应的按键
	keys = (msg->data[5] << 8) | msg->data[6];
	
	// 2号舵机控制, 用L1和R1键控制正转和反转, 
	if((keys & (1 << 14)) && (!(keys & (1 << 3))))				// L1按下且R1松开时
	{
		duty[1] -= 0.005;
		PWM(&duty[1]);
		servoPWM.servo2 = Duty_to_PWM(duty[1]);
	}
	else if((!(keys & (1 << 14))) && (keys & (1 << 3)))		// L1松开且R1按下时
	{
		duty[1] += 0.005;
		PWM(&duty[1]);
		servoPWM.servo2 = Duty_to_PWM(duty[1]);
	}
	
	// 3号舵机控制, 用L2和R2键控制正转和反转, 
	if((keys & (1 << 15)) && (!(keys & (1 << 2))))				// L2按下且R2松开时
	{
		duty[2] -= 0.005;
		PWM(&duty[2]);
		if(duty[2] < 0.1)		// 防止云台垂直舵机转动角度过大，而导致堵转
		{
			duty[2] = 0.1;
		}
		servoPWM.servo3 = Duty_to_PWM(duty[2]);
	}
	else if((!(keys & (1 << 15))) && (keys & (1 << 2)))		// L2松开且R2按下时
	{
		duty[2] += 0.005;
		PWM(&duty[2]);
		if(duty[2] > 0.9)		// 防止云台垂直舵机转动角度过大，而导致堵转
		{
			duty[2] = 0.9;
		}
		servoPWM.servo3 = Duty_to_PWM(duty[2]);
	}
	
	// 4号舵机控制, 用LL和LR键控制正转和反转, 
	if((keys & (1 << 12)) && (!(keys & (1 << 10))))				// LL按下且LR松开时
	{
		duty[3] -= 0.005;
		PWM(&duty[3]);
		servoPWM.servo4 = Duty_to_PWM(duty[3]);
	}
	else if((!(keys & (1 << 12))) && (keys & (1 << 10)))		// LL松开且LR按下时
	{
		duty[3] += 0.005;
		PWM(&duty[3]);
		servoPWM.servo4 = Duty_to_PWM(duty[3]);
	}
	
	// 5号舵机控制, 用LU和LD键控制正转和反转, 
	if((keys & (1 << 13)) && (!(keys & (1 << 11))))				// LU按下且LD松开时
	{
		duty[4] -= 0.005;
		PWM(&duty[4]);
		servoPWM.servo5 = Duty_to_PWM(duty[4]);
	}
	else if((!(keys & (1 << 13))) && (keys & (1 << 11)))		// LU松开且LD按下时
	{
		duty[4] += 0.005;
		PWM(&duty[4]);
		servoPWM.servo5 = Duty_to_PWM(duty[4]);
	}
	
	// 6号舵机控制, 用RL和RR键控制正转和反转, 
	if((keys & (1 << 7)) && (!(keys & (1 << 5))))				// RL按下且RR松开时
	{
		duty[5] -= 0.005;
		PWM(&duty[5]);
		servoPWM.servo6 = Duty_to_PWM(duty[5]);
	}
	else if((!(keys & (1 << 7))) && (keys & (1 << 5)))		// RL松开且RR按下时
	{
		duty[5] += 0.005;
		PWM(&duty[5]);
		servoPWM.servo6 = Duty_to_PWM(duty[5]);
	}
	
	// 7号舵机控制, 用RU和RD键控制正转和反转, 
	if((keys & (1 << 4)) && (!(keys & (1 << 6))))				// RU按下且RD松开时
	{
		duty[6] -= 0.005;
		PWM(&duty[6]);
		servoPWM.servo7 = Duty_to_PWM(duty[6]);
	}
	else if((!(keys & (1 << 4))) && (keys & (1 << 6)))		// RU松开且RD按下时
	{
		duty[6] += 0.005;
		PWM(&duty[6]);
		servoPWM.servo7 = Duty_to_PWM(duty[6]);
	}
	
	// 8号舵机控制, 用SE和ST键控制正转和反转, 
	if((keys & (1 << 9)) && (!(keys & (1 << 8))))					// SE按下且ST松开时
	{
		duty[7] -= 0.005;
		PWM(&duty[7]);
		servoPWM.servo8 = Duty_to_PWM(duty[7]);
	}
	else if((!(keys & (1 << 9))) && (keys & (1 << 8)))		// SE松开且ST按下时
	{
		duty[7] += 0.005;
		PWM(&duty[7]);
		servoPWM.servo8 = Duty_to_PWM(duty[7]);
	}
	
}


/*获取丢包个数*/
uint16_t radioinkFailRxcount(void)
{
	return failReceiveCount;
}

/*获取无线连接状态*/
bool radiolinkConnectStatus(void)
{
	return radioConnectStatus;
}

/*使能radiolink*/
void radiolinkEnable(FunctionalState state)
{
	if(state == ENABLE)
	{
		vTaskResume(radiolinkTaskHandle);
	}
	else
	{
		vTaskSuspend(radiolinkTaskHandle);
	}
}







