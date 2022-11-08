#ifndef __USART1_H
#define	__USART1_H


#include "stm32f10x.h"
#include "radio.h"

#define  USART1_RX_QUEUE_SIZE		10

void USART1_Init(uint32_t baudrate);
void Usart1Task(void *param);
void ProcessMsg(radioMsg_t *msg);
bool Usart1LinkStatus(void);

#endif /* __USART1_H */
