#ifndef __SERVO_H
#define	__SERVO_H


#include "stm32f10x.h"


/* 
* APB1 prescaler = 2, APB1 Frequency = 36MHz, APB1 TIMER Frequency = 72MHz
* APB2 prescaler = 1, APB2 Frequency = 72MHz, APB2 TIMER Frequency = 72MHz
*/
#define SERVO_PWM_FREQUENCE 50             // 50Hz    
#define SERVO_PWM_RESOLUTION 20000         // 20ms = 20000us
#define SERVO_DEFAULT_DUTY 1500      // 1500us
#define APB1_TIMER_CLOCKS 72000000
#define APB2_TIMER_CLOCKS 72000000
#define SERVO_TIM_PSC_APB1 ((APB1_TIMER_CLOCKS/SERVO_PWM_FREQUENCE)/SERVO_PWM_RESOLUTION -1)
#define SERVO_TIM_PSC_APB2 ((APB2_TIMER_CLOCKS/SERVO_PWM_FREQUENCE)/SERVO_PWM_RESOLUTION -1)


/*舵机PWM结构体*/
typedef struct
{
	uint16_t servo1;
	uint16_t servo2;
	uint16_t servo3;
	uint16_t servo4;
	uint16_t servo5;
	uint16_t servo6;
	uint16_t servo7;
	uint16_t servo8;
}servoPWM_t;


/**************************函数声明********************************/
void Servo_Init(void);
uint16_t Duty_to_PWM(float duty);
void Set_Servo_PWM(uint8_t channel, uint16_t pwm);
void PWM(float *duty);
void Relax_Servo(uint8_t channel);

#endif /* __SERVO_H */

