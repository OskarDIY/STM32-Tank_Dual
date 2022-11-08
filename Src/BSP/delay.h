#ifndef __DELAY_H
#define __DELAY_H 			   

#include "stm32f10x.h"

void Delay(uint16_t t);
void Delay_ns(uint16_t t);
void Delay_us(uint16_t t);
void Delay_ms(uint16_t t);

void Delay_Init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_xms(u32 nms);

#define SYSTEM_SUPPORT_OS		1

#endif





























