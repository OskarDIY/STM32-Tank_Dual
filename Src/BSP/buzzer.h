#ifndef __BUZZER_H
#define	__BUZZER_H


#include "stm32f10x.h"


#define BUZZER_ON         GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define BUZZER_OFF        GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define TOGGLE_BUZZER     GPIOA->ODR ^= GPIO_Pin_8;


void Buzzer_Init(void);
void PlayMusic(void *param);

#endif /* __BUZZER_H */
