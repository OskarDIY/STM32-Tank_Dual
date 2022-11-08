#ifndef __UART5_H
#define	__UART5_H


#include "stm32f10x.h"
#include "radio.h"

#define  UART5_RX_QUEUE_SIZE		10

void UART5_Init(uint32_t baudrate);
void Uart5Task(void *param);

#endif /* __UART5_H */
