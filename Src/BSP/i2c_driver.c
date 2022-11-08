 /**
  ******************************************************************************
  * @file    i2c_driver.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-05-02
  * @brief   i2c low level driver
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
  ******************************************************************************
  */

#include <string.h>
#include "i2c_driver.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>

// 传感器IIC总线速度
#define I2C_SENSORS_CLOCK_SPEED	400000

// Misc constants.
#define I2C_NO_BLOCK			0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2
#define I2C_MESSAGE_TIMEOUT     (1000)
//#define I2C_MESSAGE_TIMEOUT     100

// Delay is approx 0.01us per loop @72Mhz
#define I2CDEV_LOOPS_PER_US  	(100)
#define I2CDEV_LOOPS_PER_MS  	(100000)	


#define GPIO_WAIT_FOR_HIGH(gpio, pin, timeoutcycles)\
{\
	int i = timeoutcycles;\
	while(GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && i--);\
}

#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles) \
{\
	int i = timeoutcycles;\
	while(GPIO_ReadInputDataBit(gpio, pin) == Bit_SET && i--);\
}

  
/**
 * IIC底层驱动初始化
 */
static void i2cdrvInitBus(I2cDrv* i2c);
/**
 * IIC DMA初始化
 */
static void i2cdrvDmaSetupBus(I2cDrv* i2c);
/**
 * 启动IIC传输
 */
static void i2cdrvStartTransfer(I2cDrv *i2c);
/**
 * 重启IIC总线
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c);
/**
 * 环路延时
 */
static inline void i2cdrvRoughLoopDelay(uint32_t us);
/**
 * 解锁IIC总线
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA);
/**
 * 清除DMA数据流
 */
static void i2cdrvClearDMA(I2cDrv* i2c);
/**
 * 事件中断服务函数
 */
static void i2cdrvEventIsrHandler(I2cDrv* i2c);
/**
 * 错误中断服务函数
 */
static void i2cdrvErrorIsrHandler(I2cDrv* i2c);
/**
 * DMA中断服务函数
 */
static void i2cdrvDmaIsrHandler(I2cDrv* i2c);

/**
 * 传感器总线定义
 */
static const I2cDef sensorsBusDef =
{
	.i2cPort            = I2C2,
	.i2cPerif           = RCC_APB1Periph_I2C2,
	.i2cEVIRQn          = I2C2_EV_IRQn,
	.i2cERIRQn          = I2C2_ER_IRQn,
	.i2cClockSpeed      = I2C_SENSORS_CLOCK_SPEED,
	.gpioSCLPerif       = RCC_APB2Periph_GPIOB,
	.gpioSCLPort        = GPIOB,
	.gpioSCLPin         = GPIO_Pin_10,
	.gpioSCLPinSource   = GPIO_PinSource10,
	.gpioSDAPerif       = RCC_APB2Periph_GPIOB,
	.gpioSDAPort        = GPIOB,
	.gpioSDAPin         = GPIO_Pin_11,
	.gpioSDAPinSource   = GPIO_PinSource11,
//	.gpioAF             = GPIO_AF_I2C2,
	.dmaPerif           = RCC_AHBPeriph_DMA1,
//	.dmaChannel         = DMA1_Channel5,
	.dmaRxStream        = DMA1_Channel5,
	.dmaRxIRQ           = DMA1_Channel5_IRQn,
	.dmaRxTCFlag        = DMA1_FLAG_TC5,
	.dmaRxTEFlag        = DMA1_FLAG_TE5,
	
//	.dmaTxStream        = DMA1_Channel4,
//	.dmaTxIRQ           = DMA1_Channel4_IRQn,
//	.dmaTxTCFlag        = DMA1_FLAG_TC4,
//	.dmaTxTEFlag        = DMA1_FLAG_TE4,
};

/**
 * 传感器总线
 */
I2cDrv sensorsBus =
{
	.def                = &sensorsBusDef,
};

/**
 * 环路延时
 */
static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
	volatile uint32_t delay = 0;
	for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}
/**
 * 启动IIC传输
 */
static void i2cdrvStartTransfer(I2cDrv *i2c)
{
	if (i2c->txMessage.direction == i2cRead)
	{
		i2c->rxDMAStruct.DMA_BufferSize = i2c->txMessage.messageLength;
		i2c->rxDMAStruct.DMA_MemoryBaseAddr = (uint32_t)i2c->txMessage.buffer;
		DMA_Init(i2c->def->dmaRxStream, &i2c->rxDMAStruct);
		DMA_Cmd(i2c->def->dmaRxStream, ENABLE);
	}
//	else
//	{
//		i2c->txDMAStruct.DMA_BufferSize = i2c->txMessage.messageLength;
//		i2c->txDMAStruct.DMA_MemoryBaseAddr = (uint32_t)i2c->txMessage.buffer;
//		DMA_Init(i2c->def->dmaTxStream, &i2c->txDMAStruct);
//		DMA_Cmd(i2c->def->dmaTxStream, ENABLE);
//	}

	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT, ENABLE);
	i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
}

static void i2cTryNextMessage(I2cDrv* i2c)
{
	i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

static void i2cNotifyClient(I2cDrv* i2c)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(i2c->isBusFreeSemaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
 * 重启IIC总线
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}
/**
 * IIC DMA初始化
 */
static void i2cdrvDmaSetupBus(I2cDrv* i2c)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(i2c->def->dmaPerif, ENABLE);

// RX DMA Channel Config
//	i2c->DMAStruct.DMA_Channel = i2c->def->dmaChannel;
	i2c->rxDMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
	i2c->rxDMAStruct.DMA_MemoryBaseAddr = 0;
	i2c->rxDMAStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	i2c->rxDMAStruct.DMA_BufferSize = 0;
	i2c->rxDMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	i2c->rxDMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	i2c->rxDMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	i2c->rxDMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	i2c->rxDMAStruct.DMA_Mode = DMA_Mode_Normal;
	i2c->rxDMAStruct.DMA_Priority = DMA_Priority_High;
//	i2c->DMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	i2c->DMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	i2c->DMAStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	i2c->DMAStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->dmaRxIRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//	i2c->txDMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
//	i2c->txDMAStruct.DMA_MemoryBaseAddr = 0;
//	i2c->txDMAStruct.DMA_DIR = DMA_DIR_PeripheralDST;
//	i2c->txDMAStruct.DMA_BufferSize = 0;
//	i2c->txDMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	i2c->txDMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	i2c->txDMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	i2c->txDMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	i2c->txDMAStruct.DMA_Mode = DMA_Mode_Normal;
//	i2c->txDMAStruct.DMA_Priority = DMA_Priority_High;
//	
//	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->dmaTxIRQ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}
/**
 * IIC底层驱动初始化
 */
static void i2cdrvInitBus(I2cDrv* i2c)
{
	I2C_InitTypeDef  I2C_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIOB clock
	RCC_APB2PeriphClockCmd(i2c->def->gpioSDAPerif, ENABLE);
	RCC_APB2PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);
	// Enable I2C_SENSORS clock
	RCC_APB1PeriphClockCmd(i2c->def->i2cPerif, ENABLE);	

	// Configure I2C_SENSORS pins to unlock bus.
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
	GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
	GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

	i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);

	// Configure I2C_SENSORS pins for AF.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
	GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
	GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

	//Map gpios to alternate functions
//	GPIO_PinAFConfig(i2c->def->gpioSCLPort, i2c->def->gpioSCLPinSource, i2c->def->gpioAF);
//	GPIO_PinAFConfig(i2c->def->gpioSDAPort, i2c->def->gpioSDAPinSource, i2c->def->gpioAF);

	// I2C_SENSORS configuration
	I2C_DeInit(i2c->def->i2cPort);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = i2c->def->i2cClockSpeed;
	I2C_Init(i2c->def->i2cPort, &I2C_InitStructure);

	// Enable I2C_SENSORS error interrupts
	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cEVIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cERIRQn;
	NVIC_Init(&NVIC_InitStructure);

	i2cdrvDmaSetupBus(i2c);

	i2c->isBusFreeSemaphore = xSemaphoreCreateBinary();
	i2c->isBusFreeMutex = xSemaphoreCreateMutex();
}
/**
 * 解锁IIC总线
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
	GPIO_SetBits(portSDA, pinSDA);
	/* Check SDA line to determine if slave is asserting bus and clock out if so */
	while(GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET)
	{
		/* Set clock high */
		GPIO_SetBits(portSCL, pinSCL);
		/* Wait for any clock stretching to finish. */
		GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
		i2cdrvRoughLoopDelay(I2CDEV_LOOPS_PER_US);

		/* Generate a clock cycle */
		GPIO_ResetBits(portSCL, pinSCL);
		i2cdrvRoughLoopDelay(I2CDEV_LOOPS_PER_US);
		GPIO_SetBits(portSCL, pinSCL);
		i2cdrvRoughLoopDelay(I2CDEV_LOOPS_PER_US);
	}

	/* Generate a start then stop condition */
	GPIO_SetBits(portSCL, pinSCL);
	i2cdrvRoughLoopDelay(I2CDEV_LOOPS_PER_US);
	GPIO_ResetBits(portSDA, pinSDA);
	i2cdrvRoughLoopDelay(I2CDEV_LOOPS_PER_US);
	GPIO_ResetBits(portSDA, pinSDA);
	i2cdrvRoughLoopDelay(I2CDEV_LOOPS_PER_US);

	/* Set data and clock high and wait for any clock stretching to finish. */
	GPIO_SetBits(portSDA, pinSDA);
	GPIO_SetBits(portSCL, pinSCL);
	GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
	/* Wait for data to be high */
	GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, I2CDEV_LOOPS_PER_MS);
}

//-----------------------------------------------------------
/**
 * IIC外设初始化
 */
void i2cdrvInit(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

/**
 * 创建一个传输信息
 */
void i2cdrvCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      I2cDirection  direction,	
                      uint32_t length,
                      uint8_t  *buffer)
{
	message->slaveAddress = slaveAddress;
	message->direction = direction;
	message->isInternal16bit = false;
	message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
	message->messageLength = length;
	message->status = i2cAck;
	message->buffer = buffer;
	message->nbrOfRetries = I2C_MAX_RETRIES;
}

/**
 * 创建一个用于传输内部寄存器的信息
 */
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             I2cDirection  direction,	
                             uint32_t length,
                             uint8_t  *buffer)
{
	message->slaveAddress = slaveAddress;
	message->direction = direction;
	message->isInternal16bit = IsInternal16;
	message->internalAddress = intAddress;
	message->messageLength = length;
	message->status = i2cAck;
	message->buffer = buffer;
	message->nbrOfRetries = I2C_MAX_RETRIES;
}


/**
 * 发送或者接收IIC信息
 */
bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message)
{
	bool status = false;
	
	xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); // Protect message data
	
	//	Copy message
	memcpy((char*)&i2c->txMessage, (char*)message, sizeof(I2cMessage));
	//	We can now start the ISR sending this message.
	i2cdrvStartTransfer(i2c);
	
	//	Wait for transaction to be done
	if (xSemaphoreTake(i2c->isBusFreeSemaphore, I2C_MESSAGE_TIMEOUT) == pdTRUE)
	{		
		if (i2c->txMessage.status == i2cAck)
		{
			status = true;
		}
	}
	else
	{
		i2cdrvClearDMA(i2c);
		i2cdrvTryToRestartBus(i2c);
		printf("Restart I2C... \n");
	//TODO: If bus is really hanged... fail safe
	}
	
	xSemaphoreGive(i2c->isBusFreeMutex);
	return status;
}

/**
 * 事件中断服务函数
 */
static void i2cdrvEventIsrHandler(I2cDrv* i2c)
{
	uint16_t SR1;
	uint16_t SR2;

	// read the status register first
	SR1 = i2c->def->i2cPort->SR1;

	// Start bit event
	if (SR1 & I2C_SR1_SB)
	{
		i2c->messageIndex = 0;

		if(i2c->txMessage.direction == i2cWrite ||
		i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
		{
			I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1, I2C_Direction_Transmitter);
		}
		else
		{
			I2C_AcknowledgeConfig(i2c->def->i2cPort, ENABLE);
			I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1, I2C_Direction_Receiver);
		}
	}
	// Address event
	else if (SR1 & I2C_SR1_ADDR)
	{
		if(i2c->txMessage.direction == i2cWrite || i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
		{
			SR2 = i2c->def->i2cPort->SR2;                               // clear ADDR
			// In write mode transmit is always empty so can send up to two bytes
			if (i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
			{
				if (i2c->txMessage.isInternal16bit)
				{
					I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0xFF00) >> 8);
					I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
				}
				else
				{
					I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
				}
				i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
			}
			I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, ENABLE);        // allow us to have an EV7
		}
		else // Reading, start DMA transfer
		{
			if(i2c->txMessage.messageLength == 1)
			{
				I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
			}
			else
			{
				I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); // No repetitive start
			}
			// Disable buffer I2C interrupts
			I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
			// Enable the Transfer Complete interrupt
			DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, ENABLE);
			I2C_DMACmd(i2c->def->i2cPort, ENABLE); // Enable before ADDR clear

			__DMB();                         // Make sure instructions (clear address) are in correct order
			SR2 = i2c->def->i2cPort->SR2;    // clear ADDR
		}
	}
	// Byte transfer finished
	else if (SR1 & I2C_SR1_BTF)
	{
		SR2 = i2c->def->i2cPort->SR2;
		if (SR2 & I2C_SR2_TRA) // In write mode?
		{
			if (i2c->txMessage.direction == i2cRead) // internal address read
			{
				/* Internal address written, switch to read */
				i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
			}
			else
			{
				i2cNotifyClient(i2c);
				// Are there any other messages to transact? If so stop else repeated start.
				i2cTryNextMessage(i2c);
			}
		}
		else // Reading. Shouldn't happen since we use DMA for reading.
		{
//			ASSERT(1);
			i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
			if(i2c->messageIndex == i2c->txMessage.messageLength)
			{
				i2cNotifyClient(i2c);
				// Are there any other messages to transact?
				i2cTryNextMessage(i2c);
			}
		}
		// A second BTF interrupt might occur if we don't wait for it to clear.
		// TODO Implement better method.
		while (i2c->def->i2cPort->CR1 & 0x0100) { ; }
	}
	// Byte received
	else if (SR1 & I2C_SR1_RXNE) // Should not happen when we use DMA for reception.
	{
		i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
		if(i2c->messageIndex == i2c->txMessage.messageLength)
		{
			I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);   // disable RXE to get BTF
		}
	}
	// Byte ready to be transmitted
	else if (SR1 & I2C_SR1_TXE)
	{
		if (i2c->txMessage.direction == i2cRead)
		{
			// Disable TXE to flush and get BTF to switch to read.
			// Switch must be done in BTF or strange things happen.
			I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
		}
		else
		{
			I2C_SendData(i2c->def->i2cPort, i2c->txMessage.buffer[i2c->messageIndex++]);
			if(i2c->messageIndex == i2c->txMessage.messageLength)
			{
				// Disable TXE to allow the buffer to flush and get BTF
				I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
			}
			
			
//			if(i2c->txMessage.messageLength == 1)
//			{
//				I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
//			}
//			else
//			{
//				I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); // No repetitive start
//			}

//			// Disable buffer I2C interrupts
//			I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
//			// Enable the Transfer Complete interrupt
//			DMA_ITConfig(i2c->def->dmaTxStream, DMA_IT_TC | DMA_IT_TE, ENABLE);
//			I2C_DMACmd(i2c->def->i2cPort, ENABLE); // Enable before ADDR clear

//			__DMB();                         // Make sure instructions (clear address) are in correct order
//			SR2 = i2c->def->i2cPort->SR2;    // clear ADDR
			
		}
	}
}

/**
 * 错误中断服务函数
 */
static void i2cdrvErrorIsrHandler(I2cDrv* i2c)
{
	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_AF))
	{
		if(i2c->txMessage.nbrOfRetries-- > 0)
		{
			// Retry by generating start
			i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
		}
		else
		{
			// Failed so notify client and try next message if any.
			i2c->txMessage.status = i2cNack;
			i2cNotifyClient(i2c);
			i2cTryNextMessage(i2c);
		}
		I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_AF);
	}
	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_BERR))
	{
		I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_BERR);
	}
	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_OVR))
	{
		I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_OVR);
	}
	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_ARLO))
	{
		I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_ARLO);
	}
	
	
//	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_SMBALERT))
//	{
//		I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_SMBALERT);
//	}
//	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_TIMEOUT))
//	{
//		I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_TIMEOUT);
//	}
//	if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_PECERR))
//	{
//		I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_PECERR);
//	}
	
}
/**
 * 清除DMA中断标志
 */
static void i2cdrvClearDMA(I2cDrv* i2c)
{
	DMA_Cmd(i2c->def->dmaRxStream, DISABLE);
	DMA_ClearITPendingBit(i2c->def->dmaRxTCFlag);
	I2C_DMACmd(i2c->def->i2cPort, DISABLE);
	I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
	DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, DISABLE);
	
//	DMA_Cmd(i2c->def->dmaTxStream, DISABLE);
//	DMA_ClearITPendingBit(i2c->def->dmaTxTCFlag);
//	I2C_DMACmd(i2c->def->i2cPort, DISABLE);
//	I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
//	DMA_ITConfig(i2c->def->dmaTxStream, DMA_IT_TC | DMA_IT_TE, DISABLE);
}
/**
 * 事件中断服务函数
 */
static void i2cdrvDmaIsrHandler(I2cDrv* i2c)
{
	if (DMA_GetFlagStatus(i2c->def->dmaRxTCFlag)) // Tranasfer complete
	{
		i2cdrvClearDMA(i2c);
		i2cNotifyClient(i2c);
		// Are there any other messages to transact?
		i2cTryNextMessage(i2c);
	}
	if (DMA_GetFlagStatus(i2c->def->dmaRxTEFlag)) // Transfer error
	{
		DMA_ClearITPendingBit(i2c->def->dmaRxTEFlag);
		//TODO: Best thing we could do?
		i2c->txMessage.status = i2cNack;
		i2cNotifyClient(i2c);
		i2cTryNextMessage(i2c);
	}
	
//	if (DMA_GetFlagStatus(i2c->def->dmaTxTCFlag)) // Tranasfer complete
//	{
//		i2cdrvClearDMA(i2c);
//		i2cNotifyClient(i2c);
//		// Are there any other messages to transact?
//		i2cTryNextMessage(i2c);
//	}
//	if (DMA_GetFlagStatus(i2c->def->dmaTxTEFlag)) // Transfer error
//	{
//		DMA_ClearITPendingBit(i2c->def->dmaTxTEFlag);
//		//TODO: Best thing we could do?
//		i2c->txMessage.status = i2cNack;
//		i2cNotifyClient(i2c);
//		i2cTryNextMessage(i2c);
//	}
}


void I2C2_ER_IRQHandler(void)
{
	i2cdrvErrorIsrHandler(&sensorsBus);
}

void I2C2_EV_IRQHandler(void)
{
	i2cdrvEventIsrHandler(&sensorsBus);
}

void DMA1_Channel5_IRQHandler(void)
{
	i2cdrvDmaIsrHandler(&sensorsBus);
}

//void DMA1_Channel4_IRQHandler(void)
//{
//	i2cdrvDmaIsrHandler(&sensorsBus);
//}


