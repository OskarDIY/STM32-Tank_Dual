#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"
 
enum nrfMode
{
	PTX_MODE,
	PRX_MODE,
};	

enum nrfPower
{
	POWER_M18dBm,
	POWER_M12dBm,
	POWER_M6dBm,
	POWER_0dBm,
};

#define  RADIO_POWER			POWER_0dBm

enum nrfRate
{
	DATA_RATE_250K,
	DATA_RATE_1M,
	DATA_RATE_2M,
};

typedef enum 
{
	IDLE    = 0,
	MAX_RT 	= 1,
	TX_DS 	= 2,
	RX_DR 	= 3,
}nrfEvent_e;

#define FIXED_PACKET_LEN  	32		// 包长度
 
/***********************NRF SPI寄存器指令*********************************/
#define CMD_R_REG       		0x00  // 读寄存器指令
#define CMD_W_REG       		0x20  // 写寄存器指令
#define CMD_RX_PL_WID   		0x60  // 读取接收数据长度
#define CMD_R_RX_PAYLOAD 		0x61  // 读取接收数据指令
#define CMD_W_TX_PAYLOAD  	0xA0  // 写待发数据指令
#define CMD_FLUSH_TX      	0xE1  // 冲洗发送 FIFO指令
#define CMD_FLUSH_RX      	0xE2  // 冲洗接收 FIFO指令
#define CMD_REUSE_TX_PL   	0xE3  // 定义重复装载数据指令
#define CMD_NOP           	0xFF  // 保留，可用作读取status寄存器

#define CMD_W_ACK_PAYLOAD(P)  		(0xA8|(P&0x0F))	//PRX模式装载PAYLOAD并使能P通道
#define CMD_W_TX_PAYLOAD_NO_ACK 	0xB0				 		//PTX模式写NO ACK数据

//***********************NRF 寄存器地址***********************************//
#define REG_CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define REG_EN_AA           0x01  // 自动应答功能设置
#define REG_EN_RXADDR       0x02  // 可用信道设置
#define REG_SETUP_AW        0x03  // 收发地址宽度设置
#define REG_SETUP_RETR      0x04  // 自动重发功能设置
#define REG_RF_CH           0x05  // 工作频率设置
#define REG_RF_SETUP        0x06  // 发射速率、功耗功能设置
#define REG_STATUS    			0x07  // 状态寄存器
#define REG_OBSERVE_TX      0x08  // 发送监测功能
#define REG_RPD             0x09  // 接收功率检测           
#define REG_RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define REG_RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define REG_RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define REG_RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define REG_RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define REG_RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define REG_TX_ADDR         0x10  // 发送地址寄存器
#define REG_RX_PW_P0        0x11  // 接收频道0接收数据长度
#define REG_RX_PW_P1        0x12  // 接收频道1接收数据长度
#define REG_RX_PW_P2        0x13  // 接收频道2接收数据长度
#define REG_RX_PW_P3        0x14  // 接收频道3接收数据长度
#define REG_RX_PW_P4        0x15  // 接收频道4接收数据长度
#define REG_RX_PW_P5        0x16  // 接收频道5接收数据长度
#define REG_FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
#define REG_DYNPD 					0x1C  // 动态数据包长度
#define REG_FEATURE 				0x1D  // 特点寄存器

/* STATUS寄存器功能位 */
#define BIT_RX_DR (1<<6)
#define BIT_TX_DS (1<<5)
#define BIT_MAX_RT (1<<4)

#define  NRF_SPI  			SPI3
#define  NRF_CE_H()			GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define  NRF_CE_L()			GPIO_ResetBits(GPIOC, GPIO_Pin_5)

#define  SPI3_NSS_H()		GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define  SPI3_NSS_L()		GPIO_ResetBits(GPIOA, GPIO_Pin_15)

	
/* 初始化NRF24L01配置 */
void NRF_Init(enum nrfMode model);

/* 检查nrf是否通讯正常 */
ErrorStatus nrf_check(void);

/* 发送数据包(PTX模式) */
void nrf_txPacket(u8 *tx_buf,u8 len);

/* 发送NO_ACK数据包(PTX模式) */
void nrf_txPacketNoACK(u8 *tx_buf,u8 len);

/* 发送ACK数据包，设置0通道(PRX模式) */
void nrf_txPacket_AP(u8 *tx_buf,u8 len);

/* 发送NO_ACK数据包(PTX模式) */
void nrf_sendPacketNoACK(u8 *sendBuf,u8 len);
	
/* 发送数据包，并等待接收ACK (PTX模式) */
u8 nrf_sendPacketWaitACK(u8 *sendBuf ,u8 len ,
							u8 *ackBuf  ,u8 *acklen);
/* 查询事件并接收数据包 */
nrfEvent_e nrf_checkEventandRxPacket(u8 *ackBuf, u8 *acklen);

/*设置nrf中断回调函数*/
void nrf_setIterruptCallback(void(*cb)(void));

/**************************NRF24L01+配置函数***********************************/
void nrf_setAddress(uint8_t address[5]);
void nrf_setChannel(u8 channel);
void nrf_setDataRate(enum nrfRate dataRate);
void nrf_setPower(enum nrfPower power);
void nrf_setArd(void);
void nrf_setArc(u8 arc);
u8 nrf_getRpd(void);
u8 nrf_getTxRetry(void);
void nrf_txPacket(u8 *tx_buf,u8 len);
u8 nrf_rxPacket(u8 *rx_buf);
void nrf_txPacket_test(uint8_t *tx_buf, uint8_t len);

#endif


