#ifndef __LED_H
#define	__LED_H


#include "stm32f10x.h"


/* 定义LED连接的GPIO端口, 用户只需要修改下面的代码即可改变LED引脚的设置 */


// B-蓝色
#define LED_GPIO_PORT    	GPIOC			              /* GPIO端口 */
#define LED_GPIO_CLK 	    RCC_APB2Periph_GPIOC		/* GPIO端口时钟 */
#define LED_GPIO_PIN			GPIO_Pin_4			        /* GPIO引脚 */


/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* 使用标准的固件库控制IO*/
#define LED(a)	if (a)	\
					GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN)


/* 直接操作寄存器的方法控制IO */
#define GPIO_Toggle(p,i) {p->ODR ^=i;} //输出反转状态


/* 定义控制IO的宏 */
#define LED_TOGGLE		 GPIO_Toggle(LED_GPIO_PORT,LED_GPIO_PIN)
#define LED_OFF		   	 GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN)
#define LED_ON			   GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN)



void LED_Init(void);

#endif /* __LED_H */
