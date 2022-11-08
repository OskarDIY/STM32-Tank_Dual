#ifndef __CAN_H
#define	__CAN_H


#include "stm32f10x.h"

 
//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	0		  // 0,不使能;1,使能.								    
										 							 				    
uint8_t CAN_Mode_Init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);	// CAN初始化
 
uint8_t CanSendMsg(uint8_t* msg, uint8_t len);	// 发送数据

uint8_t CanReceiveMsg(uint8_t *buf);			// 接收数据


#endif /* __CAN_H */

