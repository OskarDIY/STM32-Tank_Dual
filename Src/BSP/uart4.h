#ifndef __UART4_H
#define	__UART4_H


#include "stm32f10x.h"
#include "radio.h"

#define  UART4_RX_QUEUE_SIZE		10

void UART4_Init(uint32_t baudrate);
void Uart4Task(void *param);

#endif /* __UART4_H */
