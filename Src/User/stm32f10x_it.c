/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stdbool.h"
#include "buzzer.h"
#include "usart1.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
//void HardFault_Handler(void)
//{
//  /* Go to infinite loop when Hard Fault exception occurs */
//  while (1)
//  {
//  }
//}



/**
 * @brief  This function handles Hard Fault exception.
 */
/*
void HardFault_Handler(void)
{
	//http://www.st.com/mcu/forums-cat-6778-23.html
	//****************************************************
	//To test this application, you can use this snippet anywhere:
	// //Let's crash the MCU!
	// asm (" MOVS r0, #1 \n"
	// " LDM r0,{r1-r2} \n"
	// " BX LR; \n");
	asm("PRESERVE8")
	asm("IMPORT printHardFault")
    asm("TST r14, #4")
	asm("ITE EQ")
	asm("MRSEQ R0, MSP")
	asm("MRSNE R0, PSP")
	asm("B printHardFault")
}
*/

uint32_t hard_fault_index = 0;

void  printHardFault(u32* hardfaultArgs)
{
	
	
	
	if (CoreDebug->DHCSR & 1) {  //check C_DEBUGEN == 1 -> Debugger Connected  
      __breakpoint(0);  // halt program execution here         
  }  
	
	
	
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;
	
	hard_fault_index ++;

	stacked_r0 = ((unsigned long) hardfaultArgs[0]);
	stacked_r1 = ((unsigned long) hardfaultArgs[1]);
	stacked_r2 = ((unsigned long) hardfaultArgs[2]);
	stacked_r3 = ((unsigned long) hardfaultArgs[3]);

	stacked_r12 = ((unsigned long) hardfaultArgs[4]);
	stacked_lr = ((unsigned long) hardfaultArgs[5]);
	stacked_pc = ((unsigned long) hardfaultArgs[6]);
	stacked_psr = ((unsigned long) hardfaultArgs[7]);


	printf("[Hard fault handler]\n");
	printf("R0 = %x\n", stacked_r0);
	printf("R1 = %x\n", stacked_r1);
	printf("R2 = %x\n", stacked_r2);
	printf("R3 = %x\n", stacked_r3);
	printf("R12 = %x\n", stacked_r12);
	printf("LR = %x\n", stacked_lr);
	printf("PC = %x\n", stacked_pc);
	printf("PSR = %x\n", stacked_psr);
	printf("BFAR = %x\n", (*((volatile unsigned int *)(0xE000ED38))));
	printf("CFSR = %x\n", (*((volatile unsigned int *)(0xE000ED28))));
	printf("HFSR = %x\n", (*((volatile unsigned int *)(0xE000ED2C))));
	printf("DFSR = %x\n", (*((volatile unsigned int *)(0xE000ED30))));
	printf("AFSR = %x\n", (*((volatile unsigned int *)(0xE000ED3C))));

	while (1)
	{}
}



uint32_t memory_fault_index = 0;

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
		memory_fault_index ++;
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

static uint32_t sysTickCnt=0;
extern void xPortSysTickHandler(void);

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
	{
		xPortSysTickHandler();	
	}
	else
	{
		sysTickCnt++;	/*调度开启之前计数*/
	}
}
/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之前 返回 xTaskGetTickCount()
*********************************************************/
uint32_t getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}








/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

//// 0正常模式,1文件接收模式
//extern uint8_t USART1_Mode;

//void USART1_IRQHandler(void)
//{
//	if(USART1_Mode == 0)
//	{
//		if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
//		{
//			USART_SendData(USART1, USART_ReceiveData(USART1));
//		}
//	}
//	else if(USART1_Mode == 1)
//	{
//		File_Receiver();
//	}
//}



//extern const uint16_t tone[];
//extern const uint8_t music[];
//extern const uint8_t time[];
//extern int music_len;
//extern int music_index;

//void TIM6_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//		
//		if(music_index < music_len)
//		{
//			TIM1->ARR = 1000000 / tone[music[music_index]];
//			TIM1->CCR1 = tone[music[music_index]];
//			
//			TIM6->ARR = 1000 * time[music_index];
//			TIM_SetCounter(TIM6, 0);
//			
//			music_index++;
//		}
//		else
//		{
//			TIM_SetCounter(TIM1, 0);
//			TIM_Cmd(TIM1, DISABLE);
//			TIM_CtrlPWMOutputs(TIM1, DISABLE);
//			
//			music_index = 0;
//			TIM_Cmd(TIM6, DISABLE);
//		}
//		
//		
//	}
//}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/



/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
