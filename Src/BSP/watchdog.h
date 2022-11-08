#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include <stdbool.h>
#include "stm32f10x.h"



#define WATCHDOG_RELOAD_MS 500	/*看门狗Reload时间*/
#define WATCHDOG_RESET_MS 50	/*看门狗复位时间*/
#define watchdogReset() (IWDG_ReloadCounter())


void watchdogInit(uint16_t xms);


#endif 

