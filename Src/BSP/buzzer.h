#ifndef __BUZZER_H
#define	__BUZZER_H


#include "stm32f10x.h"


#define BUZZER_ON         GPIO_SetBits(GPIOA, GPIO_Pin_12)
#define BUZZER_OFF        GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define TOGGLE_BUZZER     GPIOA->ODR ^= GPIO_Pin_12;



typedef enum
{
	MUSIC_MUTE = 0x00,					// 静音
	MUSIC_LOWBAT = 0x01,				// 低电量
	MUSIC_SHUTTING = 0x02,			// 正在关机
	MUSIC_SIGNAL_LOST = 0x03,		// 信号丢失
	MUSIC_POWER_ON = 0x04,			// 开机
}musicID_e;

void Buzzer_Init(void);
void PlayMusic(void *param);

#endif /* __BUZZER_H */
