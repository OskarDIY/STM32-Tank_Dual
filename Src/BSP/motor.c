/**
  ******************************************************************************
  * @file    motor.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-06-06
  * @brief   motor control api
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
	* 该程序仅供学习使用，未经作者允许，不得用于其它任何用途
  ******************************************************************************
  */
	
#include "can.h"
#include "radio.h"
#include "motor.h"
#include "i2c_oled.h"
#include "usart1.h"
#include "stdio.h"
#include "stm32f10x_it.h"

#include "FreeRTOS.h"
#include "task.h"

extern Menu menu;
extern servoPWM_t servoPWM;

motorStatus_t motorStatus = 
{
	.motor1_pwm = 0,					// 电机1PWM值
	.motor2_pwm = 0,					// 电机2PWM值
	.target_speed1 = 0,				// 电机1目标速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.target_speed2 = 0,				// 电机2目标速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.current_speed1 = 0,			// 电机1当前速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.current_speed2 = 0,			// 电机2当前速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.motor1_pulse = 0,				// 电机1编码器在上一个时间片内的脉冲数,即当前脉冲数,每两次采样之间的时间称为1个时间片
	.motor2_pulse = 0,				// 电机1编码器在上一个时间片内的脉冲数,即当前脉冲数,每两次采样之间的时间称为1个时间片
	.motor1_pulse_total = 0,	// 电机1脉冲总计
	.motor2_pulse_total = 0,	// 电机2脉冲总计
	.motor1_status = 0, 			// 电机1状态,0:正常,1:编码器异常
	.motor2_status = 0  			// 电机2状态,0:正常,1:编码器异常
};



// 初始化用于驱动电机1的定时器，以便向A4950电机驱动芯片输出合适的PWM信号
void TIM1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  // 输出比较通道GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为1ms
	
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_RESOLUTION;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_TIM_PSC_APB1;	
	// 时钟分频因子，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/*--------------------输出比较结构体初始化-------------------*/	
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// 输出比较通道 1
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	// 输出比较通道 4
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	//TIM_ARRPreloadConfig(TIM1, ENABLE);
	// 使能计数器
	TIM_Cmd(TIM1, ENABLE);
	
	// 主输出使能，当使用的是通用定时器时，这句不需要
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}



// 初始化用于驱动电机2的定时器，以便向A4950电机驱动芯片输出合适的PWM信号
void TIM2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
  // 输出比较通道GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 开启定时器时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为1ms
	
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_RESOLUTION;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_TIM_PSC_APB1;	
	// 时钟分频因子，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/*--------------------输出比较结构体初始化-------------------*/	
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// 输出比较通道 3
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	// 输出比较通道 4
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	//TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	// 使能计数器
	TIM_Cmd(TIM2, ENABLE);
}


// 初始化电机1的编码器,这里用了TIM5
void Motor1_Encoder_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	// 使能定时器5的时钟
	
	/*使能GPIOA和AFIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	/*初始化PA0和PA1端口为IN_FLOATING模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;			// 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;		// 设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				// 选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		// TIM向上计数  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	// 使用编码器模式
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);				// 清除TIM的更新标志位
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
  TIM_SetCounter(TIM5, 0);		// 清除计数器
  TIM_Cmd(TIM5, ENABLE); 
}


// 初始化电机2的编码器,这里用了TIM4
void Motor2_Encoder_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	// 使能定时器5的时钟
	
	/*使能GPIOB和AFIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	/*初始化PB6和PB7端口为IN_FLOATING模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;			// 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;		// 设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				// 选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		// TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	// 使用编码器模式
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);				// 清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
  TIM_SetCounter(TIM4, 0);		// 清除计数器
  TIM_Cmd(TIM4, ENABLE); 
}

int16_t motor1_cnt = 0, motor2_cnt = 0;

// 函数功能：TIM5中断服务函数
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  // 清除中断标志位
	}	    
}


// 函数功能：TIM4中断服务函数
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  // 清除中断标志位
	}
}


// 函数功能：读取单位时间内的编码器计数,用于计算电机转速
void Read_Encoder(void)
{
	// 读取电机1脉冲数,注意这里用了short,编码器正反转都不会出错
	motorStatus.motor1_pulse = -(short)TIM5 -> CNT;
	TIM5 -> CNT = 0;
	
	motorStatus.motor2_pulse = (short)TIM4 -> CNT;
	TIM4 -> CNT = 0;
}




// 将电机PWM值应用到定时器, 使定时器输出目标PWM波形
void Motor_PWM_Duty(uint8_t channel, uint16_t duty)
{
	if(channel > 4 || channel < 1)
	{
		return;
	}
	
	if(duty > 1000)
	{
		return;
	}
	
	switch(channel)
	{
		case 1: TIM1->CCR1 = duty; break;
		case 2: TIM1->CCR4 = duty; break;
		case 3: TIM2->CCR3 = duty; break;
		case 4: TIM2->CCR4 = duty; break;
	}
}


// 设置电机1的PWM值：-1000~1000, PWM值为正则正转,为负则反转,为0则停转(滑行)
void Set_Motor1_PWM(int16_t motor1)
{
	if(motor1 > 1000)
	{
		motor1 = 1000;
	}
	else if(motor1 < -1000)
	{
		motor1 = -1000;
	}
	
	// 左侧电机
	if(motor1 >= 0)				// 正转或停转
	{
		Motor_PWM_Duty(1, motor1);
		Motor_PWM_Duty(2, 0);
	}
	else									// 反转
	{
		Motor_PWM_Duty(1, 0);
		Motor_PWM_Duty(2, -motor1);
	}
}


// 设置电机2的PWM值：-1000~1000, PWM值为正则正转,为负则反转,为0则停转(滑行)
void Set_Motor2_PWM(int16_t motor2)
{
	if(motor2 > 1000)
	{
		motor2 = 1000;
	}
	else if(motor2 < -1000)
	{
		motor2 = -1000;
	}
	
	// 左侧电机
	if(motor2 >= 0)				// 正转或停转
	{
		Motor_PWM_Duty(3, 0);
		Motor_PWM_Duty(4, motor2);
	}
	else									// 反转
	{
		Motor_PWM_Duty(3, -motor2);
		Motor_PWM_Duty(4, 0);
	}
}



// 电机初始化
void Motor_Init(void)
{
	// 初始化电机PWM定时器
	TIM1_Init();
	TIM2_Init();
	
	// 关闭电机1和电机2,使其处于惯性滑行状态
	Set_Motor1_PWM(0);
	Set_Motor2_PWM(0);
	
	// 初始化电机1编码器
	Motor1_Encoder_Init();
	
	// 初始化电机2编码器
	Motor2_Encoder_Init();

}


// 阻尼刹车, 利用发电效应使电机减速
void Motor1_Brake(void)
{
	Motor_PWM_Duty(1, MOTOR_PWM_RESOLUTION);
	Motor_PWM_Duty(2, MOTOR_PWM_RESOLUTION);
}

// 阻尼刹车, 利用发电效应使电机减速
void Motor2_Brake(void)
{
	Motor_PWM_Duty(3, MOTOR_PWM_RESOLUTION);
	Motor_PWM_Duty(4, MOTOR_PWM_RESOLUTION);
}

/*
 * 根据速度计算电机PWM值,速度单位:毫米每秒
 * 这里需要用到PID闭环控制算法,对电机的速度进行控制
 *
 */
void Motor1_Speed_Control(void)
{
	// 上次的电机脉冲偏差
	static int16_t motor1_pulse_last_bias = 0;
	
	// 本次的电机脉冲偏差,用来保存目标脉冲数和当前脉冲数的偏差
	int16_t motor1_pulse_bias = 0, int16_temp = 0;
	
	/* 
	 * 根据目标线速度计算目标转速,也就是一个时间片内需要达到的脉冲数,
	 * 因为后边PID算法控制的目标变量就是电机在一个时间片内需要转过的脉冲数
	 * 线速度单位毫米每秒,转速单位为脉冲数每时间片
	 */
	int16_t motor1_pulse_target = 0;
	
	// 判断是否要进入惯性滑行或者阻尼刹车状态
	// target_speed的最高2位用来含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint8_t motor1_mode = 0;
	int8_t motor1_sign = 1;
	
	motor1_mode = motorStatus.target_speed1 >> 14;
	
	// 根据不同状态作出相应处理
	if(motor1_mode == 0)				// 惯性滑行
	{
		motorStatus.motor1_pwm = 0;
		Set_Motor1_PWM(motorStatus.motor1_pwm);
		return;
	}
	else if(motor1_mode == 3)		// 阻尼刹车
	{
		motorStatus.motor1_pwm = 0;
		Motor1_Brake();
		return;
	}
	else if(motor1_mode == 1)		// 正转
	{
		motor1_sign = 1;
	}
	else if(motor1_mode == 2)		// 反转
	{
		motor1_sign = -1;
	}
	
	// 目标脉冲数
	motor1_pulse_target = motor1_sign * ((motorStatus.target_speed1 & 0x3FFF) / WHEEL_PERIMETER * MOTOR_PULSE) / (1000 / MOTOR_TIME_SLICE);
	
//	printf("t:%d, p:%d\n", motor1_pulse_target, motorStatus.motor1_pulse);
	
	// 计算脉冲偏差
	motor1_pulse_bias = motor1_pulse_target - motorStatus.motor1_pulse;
	
	// 计算PWM值
	int16_temp = motorStatus.motor1_pwm + motor1_pulse_bias * MOTOR_PID_P - motor1_pulse_last_bias * MOTOR_PID_D;
	if(int16_temp > 1000)
	{
		int16_temp = 1000;
	}
	else if(int16_temp < -1000)
	{
		int16_temp = -1000;
	}
	
	motorStatus.motor1_pwm = int16_temp;
	
	motor1_pulse_last_bias = motor1_pulse_bias;
	
	Set_Motor1_PWM(motorStatus.motor1_pwm);
}

/*
 * 根据速度计算电机PWM值,速度单位:毫米每秒
 * 这里需要用到PID闭环控制算法,对电机的速度进行控制
 *
 */
void Motor2_Speed_Control(void)
{
	// 上次的电机脉冲偏差
	static int16_t motor2_pulse_last_bias = 0;
	
	// 本次的电机脉冲偏差,用来保存目标脉冲数和当前脉冲数的偏差
	int16_t motor2_pulse_bias = 0, int16_temp = 0;
	
	/* 
	 * 根据目标线速度计算目标转速,也就是一个时间片内需要达到的脉冲数,
	 * 因为后边PID算法控制的目标变量就是电机在一个时间片内需要转过的脉冲数
	 * 线速度单位毫米每秒,转速单位为脉冲数每时间片
	 */
	int16_t motor2_pulse_target = 0;
	
	// 判断是否要进入惯性滑行或者阻尼刹车状态
	// target_speed的最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint8_t motor2_mode = 0;
	int8_t motor2_sign = 1;
	
	motor2_mode = motorStatus.target_speed2 >> 14;
	
	// 根据不同状态作出相应处理
	if(motor2_mode == 0)				// 惯性滑行
	{
		motorStatus.motor2_pwm = 0;
		Set_Motor2_PWM(motorStatus.motor2_pwm);
		return;
	}
	else if(motor2_mode == 3)		// 阻尼刹车
	{
		motorStatus.motor2_pwm = 0;
		Motor2_Brake();
		return;
	}
	else if(motor2_mode == 1)		// 正转
	{
		motor2_sign = 1;
	}
	else if(motor2_mode == 2)		// 反转
	{
		motor2_sign = -1;
	}
	
	// 目标脉冲数
	motor2_pulse_target = motor2_sign * ((motorStatus.target_speed2 & 0x3FFF) / WHEEL_PERIMETER * MOTOR_PULSE) / (1000 / MOTOR_TIME_SLICE);
	
//	printf("t:%d, p:%d\n", motor2_pulse_target, motorStatus.motor2_pulse);
	
	// 计算脉冲偏差
	motor2_pulse_bias = motor2_pulse_target - motorStatus.motor2_pulse;
	
	// 计算PWM值
	int16_temp = motorStatus.motor2_pwm + motor2_pulse_bias * MOTOR_PID_P - motor2_pulse_last_bias * MOTOR_PID_D;
	if(int16_temp > 1000)
	{
		int16_temp = 1000;
	}
	else if(int16_temp < -1000)
	{
		int16_temp = -1000;
	}
	
	motorStatus.motor2_pwm = int16_temp;
	
	motor2_pulse_last_bias = motor2_pulse_bias;
	
	Set_Motor2_PWM(motorStatus.motor2_pwm);
}

// 计算电机当前输出线速度,单位毫米每秒
void Motor_Speed_Calculate(void)
{
	int16_t m = 0;
	m = ((float)(motorStatus.motor1_pulse * (1000 / MOTOR_TIME_SLICE)) / MOTOR_PULSE * WHEEL_PERIMETER);
	if(m > 0)
	{
		motorStatus.current_speed1 = (1 << 14) | (m & 0x3FFF);
	}
	else if(m < 0)
	{
		motorStatus.current_speed1 = (2 << 14) | (m & 0x3FFF);
	}
	else if(m == 0)
	{
		motorStatus.current_speed1 = motorStatus.target_speed1 & 0xC000;
	}
	
	m = ((float)(motorStatus.motor2_pulse * (1000 / MOTOR_TIME_SLICE)) / MOTOR_PULSE * WHEEL_PERIMETER);
	if(m > 0)
	{
		motorStatus.current_speed2 = (1 << 14) | (m & 0x3FFF);
	}
	else if(m < 0)
	{
		motorStatus.current_speed2 = (2 << 14) | (m & 0x3FFF);
	}
	else if(m == 0)
	{
		motorStatus.current_speed2 = motorStatus.target_speed2 & 0xC000;
	}
	
}

extern uint8_t can_buf[8];

void MotorTask(void *param)
{
	while(menu == RUNNING)
	{
		vTaskDelay(100);
	}
	
//	Motor_Init();
	
	while(1)
	{
		if(menu == MPU_INIT || menu == SHUTDOWN || menu == START)
		{
			Set_Motor1_PWM(0);
			Set_Motor2_PWM(0);
			vTaskDelay(50);
		}
		else
		{
			Read_Encoder();
			Motor_Speed_Calculate();
			
			if(radiolinkConnectStatus())		// 检查是否有手柄无线连接,如果手柄已经连接,则按照手柄指令运行,忽略其他连接
			{
				Set_Motor1_PWM(motorStatus.motor1_pwm);
				Set_Motor2_PWM(motorStatus.motor2_pwm);
				Set_Servo_PWM(1, servoPWM.servo1);
				Set_Servo_PWM(2, servoPWM.servo2);
				Set_Servo_PWM(3, servoPWM.servo3);
				Set_Servo_PWM(4, servoPWM.servo4);
				Set_Servo_PWM(5, servoPWM.servo5);
				Set_Servo_PWM(6, servoPWM.servo6);
				Set_Servo_PWM(7, servoPWM.servo7);
				Set_Servo_PWM(8, servoPWM.servo8);
			}
			else if(Usart1LinkStatus())			// 检查USART1是否有来自上位机的数据连接,如果有则执行上位机指令
			{
				Motor1_Speed_Control();
				Motor2_Speed_Control();
				Set_Servo_PWM(1, servoPWM.servo1);
				Set_Servo_PWM(2, servoPWM.servo2);
				Set_Servo_PWM(3, servoPWM.servo3);
				Set_Servo_PWM(4, servoPWM.servo4);
				Set_Servo_PWM(5, servoPWM.servo5);
				Set_Servo_PWM(6, servoPWM.servo6);
				Set_Servo_PWM(7, servoPWM.servo7);
				Set_Servo_PWM(8, servoPWM.servo8);
			}
			else														// 无线和上位机数据连接都已断开,关闭电机输出
			{
				motorStatus.motor1_pwm = 0;
				motorStatus.motor2_pwm = 0;
				Set_Motor1_PWM(0);
				Set_Motor2_PWM(0);
			}
			vTaskDelay(MOTOR_TIME_SLICE);
		}
		
		CanSendMsg(can_buf,8);//发送8个字节
		
	}
}

/*********************************************END OF FILE**********************/
