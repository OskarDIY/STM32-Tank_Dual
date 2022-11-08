#ifndef __KEY_H
#define	__KEY_H


#include "stm32f10x.h"

#define POWER_KEY_STATUS	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)

void UserKey_Init(void);
void PowerKey_Init(void);

#endif /* __KEY_H */
