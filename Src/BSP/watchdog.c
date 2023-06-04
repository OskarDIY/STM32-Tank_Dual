 /**
  ******************************************************************************
  * @file    watchdog.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-05-02
  * @brief   watchdog
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Blog: www.mindsilicon.com
  *
  ******************************************************************************
  */



#include "watchdog.h"





void watchdogInit(uint16_t xms)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* 40000/32Hz => 1.25  1ms*/
	IWDG_SetReload((uint16_t)(1.25*xms));

	watchdogReset();
	IWDG_Enable();
}
