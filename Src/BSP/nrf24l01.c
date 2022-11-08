/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-06-01
  * @brief   nrf24l01 driver
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
	* 该程序仅供学习使用，未经作者允许，不得用于其它任何用途
  ******************************************************************************
  */

#include "radio.h"
#include "nrf24l01.h"
#include <stdio.h>

static void (*interruptCb)(void) = 0;

/***************************NRF24L01+驱动函数***********************************/

/* NRF初始化，使用STM32的SPI3 */
static void NRF_LowLevel_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); 

	/* 配置SPI3的SCK(PB3),MISO(PB4),MOSI(PB5)引脚 */ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 配置NRF的CE(PC5),NSS(PA15)引脚 */ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 配置NRF的IRQ引脚(PC13) */ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* 配置外IRQ外部中断 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
  
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	/* 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工 */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;													/* 设置SPI工作模式:设置为主SPI */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;											/* 设置SPI的数据大小:SPI发送接收8位帧结构 */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;														/* 选择了串行时钟的稳态:时钟悬空低 */
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;													/* 数据捕获于第一个时钟沿 */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;															/* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		/*定义波特率预分频的值:波特率预分频值为4 */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;										/* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
	SPI_InitStructure.SPI_CRCPolynomial = 7;															/* CRC值计算的多项式 */
	SPI_Init(SPI3, &SPI_InitStructure);  																	/* 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器 */
 
	SPI_Cmd(SPI3, ENABLE);	/*使能SPI外设*/
	
	SPI3_NSS_H();
	NRF_CE_L();
}

static uint8_t SPI_RWByte(SPI_TypeDef* SPIx , uint8_t TxData)
{			
	/* 通过外设SPIx发送一个数据 */
	SPI_I2S_SendData(SPIx, TxData);
	/* 检查指定的SPI标志位设置与否:发送缓存空标志位*/
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 检查指定的SPI标志位设置与否:接受缓存非空标志位 */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* 返回通过SPIx最近接收的数据 */
	return SPI_I2S_ReceiveData(SPIx); 	
}

/* 写寄存器 */
static uint8_t writeReg(uint8_t reg, uint8_t value)
{
	uint8_t status;	
	SPI3_NSS_L();                	 	
	status = SPI_RWByte(NRF_SPI, reg | CMD_W_REG);
	SPI_RWByte(NRF_SPI, value); 	
	SPI3_NSS_H();                 	  
	return status;       					
}

/* 读寄存器 */ 
static uint8_t readReg(uint8_t reg)
{
	uint8_t reg_val;	    
 	SPI3_NSS_L();         		 			
	SPI_RWByte(NRF_SPI, reg | CMD_R_REG);
	reg_val = SPI_RWByte(NRF_SPI, 0xA5);
	SPI3_NSS_H();     								    
	return reg_val;    						
}	

/* 读缓冲区 */
static uint8_t readBuf(uint8_t cmd, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, i;
	SPI3_NSS_L();            
	status = SPI_RWByte(NRF_SPI, cmd);
	
	for(i = 0; i < len; i++)
	{
		pBuf[i] = SPI_RWByte(NRF_SPI, 0XFF);
	}
	SPI3_NSS_H();
	return status;
}

/* 写缓冲区 */
static uint8_t writeBuf(uint8_t cmd, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, i;	    
	SPI3_NSS_L();          
	status = SPI_RWByte(NRF_SPI, cmd);
	
	for(i = 0; i < len; i++)
	{
		SPI_RWByte(NRF_SPI, *pBuf++);
	}
	SPI3_NSS_H();
	return status;  
}

/* 发送数据包(PTX模式) */
void nrf_txPacket(uint8_t *tx_buf, uint8_t len)
{	
	NRF_CE_L();	
	writeBuf(CMD_W_TX_PAYLOAD,tx_buf,32);
	NRF_CE_H();		 
	
	
//	uint8_t reg;
//	uint8_t txdata[32] = "sdfhfgghhgfddfgdfdhggh";
//	reg = readReg(REG_CONFIG);
//	writeReg(REG_CONFIG, reg & ~1 );
//	writeReg(CMD_FLUSH_TX,0xff);		/* 冲洗TX_FIFO */
//	NRF_CE_L();	
//	writeBuf(CMD_W_TX_PAYLOAD,tx_buf,32);
//	NRF_CE_H();	
//	
//	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13));
//	
//	reg = readReg(REG_STATUS);
//	writeReg(REG_STATUS ,reg);
	
}


/* 发送数据包(PTX模式) */
void nrf_txPacket_test(uint8_t *tx_buf, uint8_t len)
{
	uint8_t reg;
	uint8_t txdata[32] = "sdfhfgghhgfddfgdfdhggh";
	reg = readReg(REG_CONFIG);
	writeReg(REG_CONFIG, reg & ~1 );
	writeReg(CMD_FLUSH_TX,0xff);		/* 冲洗TX_FIFO */
	NRF_CE_L();	
	writeBuf(CMD_W_TX_PAYLOAD,txdata,32);
	NRF_CE_H();
	
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13));
	
	reg = readReg(REG_STATUS);
	writeReg(REG_STATUS ,reg);
}




/* 发送NO_ACK数据包(PTX模式) */
void nrf_txPacketNoACK(uint8_t *tx_buf,uint8_t len)
{	
	NRF_CE_L();		 
	writeBuf(CMD_W_TX_PAYLOAD_NO_ACK,tx_buf,len);
	NRF_CE_H();		
}

/* 发送ACK数据包，设置0通道(PRX模式) */
void nrf_txPacket_AP(uint8_t *tx_buf,uint8_t len)
{	
	NRF_CE_L();		 	
	writeBuf(CMD_W_ACK_PAYLOAD(0),tx_buf,len);
	NRF_CE_H();		 
}

/* 发送NO_ACK数据包(PTX模式) */
void nrf_sendPacketNoACK(uint8_t *sendBuf,uint8_t len)
{	
	while((readReg(REG_STATUS)&0x01)!=0);	/* 等待TX_FIFO不为full */
	nrf_txPacketNoACK(sendBuf,len);			/* 发送NO_ACK数据包 */
}

/**************************NRF24L01+配置函数***********************************/
/* 设置发射接收地址，这里收发地址一致 */
void nrf_setAddress(uint8_t address[5])
{
	writeReg(REG_SETUP_AW, 0x03);											// 设置地址宽度为5字节
	writeBuf(CMD_W_REG | REG_RX_ADDR_P0, address, 5);	// 接收使用P0节点
	writeBuf(CMD_W_REG | REG_TX_ADDR, address, 5); 		
}

/* 设置频率通道,channel:0~125 */
void nrf_setChannel(uint8_t channel)
{
	if(channel <= 125)
	{		
		writeReg(REG_RF_CH, channel);
	}
}

/* 设置传输速率，dr:0->250KHz、1->1MHz、2->2MHz。 */
void nrf_setDataRate(enum nrfRate dataRate)
{
	uint8_t reg_rf = readReg(REG_RF_SETUP);
	reg_rf &= ~((1<<5)|(1<<3));/* 清除原设速率 */
	switch(dataRate)
	{
		case DATA_RATE_250K:
			reg_rf |= 0x20;
			break;
		case DATA_RATE_1M:
			reg_rf |= 0x00;
			break;
		case DATA_RATE_2M:
			reg_rf |= 0x08;
			break;	
	}
	writeReg(REG_RF_SETUP,reg_rf); 	
}

/* 设置发射功率,power: 0->-18dB  1->-12dB  2->-6dB  3->0dB */
void nrf_setPower(enum nrfPower power)
{
	uint8_t reg_rf = readReg(REG_RF_SETUP);
	reg_rf &= 0xF8;/* 清除原设功率 */
	switch(power)
	{
		case POWER_M18dBm:
			reg_rf |= 0x00;
			break;
		case POWER_M12dBm:
			reg_rf |= 0x02;
			break;
		case POWER_M6dBm:
			reg_rf |= 0x04;
			break;
		case POWER_0dBm:
			reg_rf |= 0x07;
			break;	
	}
	writeReg(REG_RF_SETUP,reg_rf);
}

/* 设置重发时间间隔，根据速率及收发字节大小设置 */
/* 详细说明参考nrf24l01.datasheet的P34. */
void nrf_setArd(void)
{
	uint8_t reg_rf, reg_retr;
	reg_rf = readReg(REG_RF_SETUP);
	reg_retr = readReg(REG_SETUP_RETR);
	
	if(!(reg_rf&0x20))	/* 速率不是250K(寄存器0x20) */
	{
		reg_retr|= 1<<4;/* (1+1)*250=500us,在接收32字节时 */
	}
	else
	{
		reg_retr|= 5<<4;/* (5+1)*250=1500us,在接收32字节时 */
	}
	
	writeReg(REG_SETUP_RETR,reg_retr);
}

/* 设置重发次数，arc:0~15 */
void nrf_setArc(uint8_t arc)
{
	uint8_t reg_retr;
	if(arc > 15)
	{
		return;
	}
	
	reg_retr = readReg(REG_SETUP_RETR);
	reg_retr |= arc;
	writeReg(REG_SETUP_RETR, reg_retr);
}

/* 获取接收功率检测 */
uint8_t nrf_getRpd(void)
{
   return readReg(REG_RPD);
}

/* 获取重发失败次数 */
uint8_t nrf_getTxRetry(void)
{
   return readReg(REG_OBSERVE_TX)&0x0F;
}


extern systemConfig_t configParam;

/* 初始化NRF24L01配置 */
/* model: PTX_MODE、PRX_MODE */
void NRF_Init(enum nrfMode model)
{
	NRF_LowLevel_Init();
	nrf_setAddress(configParam.radio.addr);
	nrf_setChannel(configParam.radio.channel);
	nrf_setDataRate(configParam.radio.dataRate);
	nrf_setPower(RADIO_POWER);		// 发射功率
	nrf_setArd();									// 重发时间间隔设置
	nrf_setArc(3);								// 重发次数
	
	if(model == PRX_MODE)
	{
		writeReg(REG_CONFIG, 0x0f);   	/* IRQ收发完成中断开启,16位CRC,PRX */
		writeReg(REG_DYNPD,0x01);				/* 使能RX_P0动态长度PLAYLOAD */
		writeReg(REG_FEATURE,0x06);			/* 使能动态长度PLAYLOAD、发送ACK PLAYLOAD */
		
		writeReg(REG_EN_AA,0x01); 			/* 使能通道0的自动应答 */	
		
		writeReg(CMD_FLUSH_TX,0xff);		/* 冲洗TX_FIFO */
		writeReg(CMD_FLUSH_RX,0xff);
	}
	else							 	
	{
		writeReg(REG_CONFIG, 0x0E);			// TX Mode, Power ON, 2Bytes CRC, Enable CRC, IRQ 收发完成中断开启
		writeReg(REG_DYNPD, 0x01);				// 使能RX_P0动态长度PLAYLOAD
		writeReg(REG_FEATURE, 0x06);		// 使能动态长度、ACK PLAYLOAD发送、W_TX_PAYLOAD_NOACK
		
		writeReg(CMD_FLUSH_TX, 0xFF);		// 冲洗TX_FIFO
		writeReg(CMD_FLUSH_RX, 0xFF);
	}										
}

/* 检查MCU与24l01是否通讯正常 */
/* 方法：写入读出地址是否一致 */

ErrorStatus nrf_check(void)
{ 
	uint8_t addr[5] = {0x11, 0x22, 0x33, 0x44, 0x55}, read_addr[5], i;
	
	NRF_LowLevel_Init();
	writeBuf(CMD_W_REG | REG_TX_ADDR, addr, 5); 
	readBuf(CMD_R_REG | REG_TX_ADDR, read_addr, 5); 
	
	for( i = 0; i < 5; i++ )
	{
		if( addr[ i ] != read_addr[ i ] )
		{
			return ERROR;
		}	
	} 
	
	return SUCCESS;
}

/* 接收数据包，返回包长度len */
uint8_t nrf_rxPacket(uint8_t *rx_buf)
{	
	uint8_t rx_len = readReg(CMD_RX_PL_WID);
	if(rx_len>0 && rx_len<33)
	{
		NRF_CE_L();	
		readBuf(CMD_R_RX_PAYLOAD,rx_buf,rx_len);
		NRF_CE_H();
	}
	else 
		rx_len = 0;
	writeReg(CMD_FLUSH_RX,0xff);/* 冲洗RX_FIFO */
	return rx_len;		
}

/* 查询事件并接收数据包 */
nrfEvent_e nrf_checkEventandRxPacket(uint8_t *ackBuf, uint8_t *acklen)
{
	nrfEvent_e nrfEvent = IDLE;
	*acklen = 0;
	uint8_t status = readReg(REG_STATUS);/*读事件标志寄存器*/
	
//	printf("Radio task nrfEvent_e: %d\n", status);
	
	if(status&BIT_MAX_RT)/*重发失败*/
	{
		writeReg(CMD_FLUSH_TX,0xff);
		nrfEvent =  MAX_RT;
	}
	else if(status&BIT_RX_DR)/*接收数据到RX_FIFO*/
	{
		*acklen =  nrf_rxPacket(ackBuf);
		nrfEvent = RX_DR;
	}
	else if(status&BIT_TX_DS)/*发送数据至TX_FIFO成功*/
	{
		nrfEvent = TX_DS;
	}
	writeReg(REG_STATUS,0x70);/*清除标志*/
	uint8_t status1 = readReg(REG_STATUS);/*读事件标志寄存器*/
	status1 = status1;
	return nrfEvent;
}

/* 发送数据包，并等待接收ACK(PTX模式) */
/* 返回值：1成功、0失败*/
uint8_t nrf_sendPacketWaitACK(uint8_t *sendBuf, uint8_t len, uint8_t *ackBuf, uint8_t *acklen)
{ 
	if(len == 0) return 0;
	nrf_txPacket(sendBuf,len);
	while((readReg(REG_STATUS)&0x70) == 0);/* 等待事件 */
	nrfEvent_e nrfEvent = nrf_checkEventandRxPacket(ackBuf, acklen);
	if(nrfEvent == MAX_RT)
		return 0;
	return 1;
}

/*设置nrf中断回调函数*/
void nrf_setIterruptCallback(void(*cb)(void))
{
	interruptCb = cb;
}

/*外部中断服务函数*/
void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line13) == SET)
	{
		if(interruptCb)
		{
			interruptCb();
		}
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
}

