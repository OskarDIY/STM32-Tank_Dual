#ifndef __POWER_CONTROL_H
#define	__POWER_CONTROL_H

#include "stm32f10x.h"


#define ADJ_OUTPUT_ON				GPIO_SetBits(GPIOC, GPIO_Pin_15)
#define ADJ_OUTPUT_OFF			GPIO_ResetBits(GPIOC, GPIO_Pin_15)

#define MAIN_OUTPUT_ON			GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define MAIN_OUTPUT_OFF			GPIO_ResetBits(GPIOC, GPIO_Pin_14)

#define POWER_ON						GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define POWER_OFF						GPIO_ResetBits(GPIOC, GPIO_Pin_0)


void ADJ_Outpt_Init(void);
void MAIN_Output_Init(void);
void PowerControl_Init(void);

void PowerTask(void *param);

#endif /* __POWER_CONTROL_H */
