#include "delay.h"

// 如果使用OS,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"	
#include "task.h"	  
#endif

void Delay(uint16_t t)
{
	while(t--);
	return;
}

void Delay_ns(uint16_t t)
{
	while(t--);
	return;
}

void Delay_us(uint16_t t) 
{  
   while(t--) 
	{
      Delay(6);    
   }	
}

void Delay_ms(uint16_t t) 
{ 
   while(t--) 
		{
      Delay_us(1000);    
   }
}

// us延时倍乘数
static uint32_t fac_us = 0;

// ms延时倍乘数,在os下,代表每个节拍的ms数
#if SYSTEM_SUPPORT_OS		
    static uint16_t fac_ms = 0;
#endif

//extern void xPortSysTickHandler(void);
//systick中断服务函数,使用OS时用到
//void SysTick_Handler(void)
//{	
//	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
//    {
//        xPortSysTickHandler();	
//    }
//}

			   
// 初始化延迟函数
// 当使用OS的时候,此函数会初始化OS的时钟节拍
// SYSTICK的时钟固定为AHB时钟的1/8
// SYSCLK: 系统时钟频率
void Delay_Init(uint8_t SYSCLK)
{
//如果需要支持OS
#if SYSTEM_SUPPORT_OS
	uint32_t reload;
#endif
	
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	// 不论是否使用OS, fac_us都需要使用
	fac_us = SYSCLK;

//如果需要支持OS
#if SYSTEM_SUPPORT_OS
	// 每秒钟的计数次数,单位为M	   
	reload = SYSCLK;
	
	// 根configTICK_RATE_HZ定溢出时间
	// reload为24位寄存器,最大值:16777216, 在72M下, 约合0.233017s左右	
	reload *= 1000000 /configTICK_RATE_HZ;
	
	fac_ms = 1000/configTICK_RATE_HZ;			//代表OS可以延时的最少单位	   
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD = reload; 					//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开启SYSTICK    
#else
	fac_ms = (uint16_t)fac_us * 1000;				//非OS下,代表每个ms需要的systick时钟数   
#endif
}

#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
//延时nus
//nus:要延时的us数.	
//nus:0~44739242(最大值即2^32/fac_us@fac_us=96)	    								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;				//LOAD的值	    	 
	ticks = nus * fac_us; 						//需要的节拍数 
	told = SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow = SysTick->VAL;	
		if(tnow != told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};											    
}


//延时nms
//nms:要延时的ms数
//nms:0~65535
void delay_ms(uint16_t nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行	    
	{		 
		if(nms >= fac_ms)						//延时的时间大于OS的最少时间周期 
		{ 
   			vTaskDelay(nms / fac_ms);	 		//FreeRTOS延时
		}
		nms %= fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
	}
	
	delay_us((uint32_t)(nms * 1000));				//普通方式延时
}
#else  //不用ucos时
//延时nus
//nus为要延时的us数.	
//注意:nus的值,不要大于174762us(最大值即2^24/fac_us@fac_us=96)
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD = nus * fac_us; 				     //时间加载	  		 
	SysTick->VAL = 0x00;        				       //清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; //开始倒数
	
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	     //等待时间到达
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL = 0X00;       				         //清空计数器 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对168M条件下,nms<=798ms 
//void delay_xms(u16 nms)
//{	 		  	  
//	u32 temp;		   
//	SysTick->LOAD=(u32)nms*fac_ms;			//时间加载(SysTick->LOAD为24bit)
//	SysTick->VAL =0x00;           			//清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数 
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;     		  		//清空计数器	  	    
//} 

//延时nms 
//nms:0~65535
void delay_ms(uint16_t nms)
{	 	 
	uint8_t repeat = nms / 540;						//这里用540,是考虑到某些客户可能超频使用,
											//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
	uint16_t remain = nms % 540;
	
	while(repeat)
	{
		delay_xms(540);
		repeat--;
	}
	
	if(remain)
		delay_xms(remain);
} 
#endif

//延时nms,不会引起任务调度
//nms:要延时的ms数
void delay_xms(uint32_t nms)
{
	uint32_t i;
	for(i = 0; i < nms; i++)
	{
		delay_us(1000);
	}
}			 
















































