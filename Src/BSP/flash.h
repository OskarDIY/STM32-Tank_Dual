#ifndef __FLASH_H
#define	__FLASH_H


#include "stm32f10x.h"
#include <stdbool.h>


/* STM32大容量产品每页大小2KByte，中、小容量产品每页大小1KByte */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)	//2048
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)	//1024
#endif

//写入的起始地址与结束地址
#define WRITE_START_ADDR  ((uint32_t)0x0803F800)
#define WRITE_END_ADDR    ((uint32_t)0x0803FFFF)

#define CONFIG_DATA_SIZE 12


void LoadConfigData(void);
bool SaveConfigData(void);


#endif /* __FLASH_H */

