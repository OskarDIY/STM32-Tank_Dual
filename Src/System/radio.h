#ifndef __RADIO_H
#define	__RADIO_H

#include "stm32f10x.h"
#include <stdbool.h>
#include "nrf24l01.h"



/* 默认配置参数 */
#define  VERSION	11		/*表示版本为V1.1*/
#define  DISPLAY_LANGUAGE	ENGLISH

#define  RADIO_CHANNEL 		0
#define  RADIO_DATARATE 	DATA_RATE_250K
#define  RADIO_ADDRESS 		{0x11, 0x22, 0x33, 0x44, 0x55}

enum language
{
	SIMPLE_CHINESE,
	ENGLISH,
	COMPLEX_CHINESE,
};

/*无线配置结构*/
typedef struct
{
	uint8_t channel;		
	enum nrfRate dataRate;
	uint8_t addr[5];
}radioConfig_t;


/*保存参数结构*/
typedef struct
{
	uint8_t version;				/*软件版本号*/
	enum language language;	/*显示语言*/
	radioConfig_t radio;		/*无线配置参数*/
	uint8_t cksum;					/*校验*/
	uint32_t fs_mark;				// file system mark, 文件系统标记, 用来判断外置spi flash内是否已经创建过文件系统
													// 如果已经创建过文件系统, fs_mark会被设置为0xAABBCCDD, 此时
													// 应该禁止格式化spi flash芯片, 以免丢失芯片内的资源文件
}systemConfig_t;


 /*
 通信命令格式
 帧头（2字节） 通信代码（1字节） 数据长度（不含帧头、通信代码和校验） 数据内容（若干字节） 校验（2字节）
 例如
 0xAA 0xAA 0x01 0x08 nByteData 0x__ 0x__
 */
#define RADIO_MSG_MAX_SIZE 32

#define HEADER_BYTE1 0xAA
#define HEADER_BYTE2 0xAA


typedef enum
{
	CMD_UNDEF = 0x00,		// 未定义命令,预留
	// Control Board即控制板,因为接收机很多时候时装在或者集成在控制板上的,所以常用Contol Board指代接收机Receiver
	
	/* ***********手柄和接收机之间的通信********** */
	// Command, Remote Controller to Control Board (Receiver),从手柄发往接收机的命令数据
	CMD_RTCB1 = 0x01,		// 从遥控手柄发送到接收机的命令1, 用于发送所有摇杆和按键的状态数据
	CMD_RTCB2 = 0x02,		// 从遥控手柄发送到接收机的命令2
	CMD_RTCB3 = 0x03,		// 从遥控手柄发送到接收机的命令3
	CMD_RTCB4 = 0x04,		// 从遥控手柄发送到接收机的命令4
	CMD_RTCB5 = 0x05,		// 从遥控手柄发送到接收机的命令5
	CMD_RTCB6 = 0x06,		// 从遥控手柄发送到接收机的命令6
	CMD_RTCB7 = 0x07,		// 从遥控手柄发送到接收机的命令7
	CMD_RTCB8 = 0x08,		// 从遥控手柄发送到接收机的命令8
	
	// Response, Control Board (Receiver) to Remote Controller,从接收机发往手柄的反馈数据
	RES_RTCB1 = 0x09,		// 接收机对RTCB1的响应, 用于回传数据触发震动
	RES_RTCB2 = 0x0A,		// 接收机对RTCB2的响应, 用于发送数据给上位机
	RES_RTCB3 = 0x0B,		// 接收机对RTCB3的响应
	RES_RTCB4 = 0x0C,		// 接收机对RTCB4的响应
	RES_RTCB5 = 0x0D,		// 接收机对RTCB5的响应
	RES_RTCB6 = 0x0E,		// 接收机对RTCB6的响应
	RES_RTCB7 = 0x0F,		// 接收机对RTCB7的响应
	RES_RTCB8 = 0x10,		// 接收机对RTCB8的响应
	
	
	/* ***********设置命令,用于上位机设置接收机,手柄或控制板,设置时被设置的设备需要用USB和上位机直连********** */
	// Command, APP to Control Board (Receiver), 从上位机发送到接收机,手柄或者控制板
	CMD_APPTCB1 = 0x11,	// 从上位机发送到接收机或手柄的命令1, 读取地址和频率
	CMD_APPTCB2 = 0x12,	// 从上位机发送到接收机或手柄的命令2, 写入地址
	CMD_APPTCB3 = 0x13,	// 从上位机发送到接收机或手柄的命令3, 写入频率
	CMD_APPTCB4 = 0x14,	// 从上位机发送到        手柄的命令4, 校准手柄, 进入校准第一阶段, 进入校准第二阶段, 保存校准结果并退出校准
	CMD_APPTCB5 = 0x15,	// 从上位机发送到接收机或手柄的命令5, 透传功能, 上位机发送数据给接收机
	CMD_APPTCB6 = 0x16,	// 从上位机发送到接收机      的命令6, 设置8路舵机PWM值和电机转速,电机转速弧度每秒,开启2路电源对外输出
	CMD_APPTCB7 = 0x17,	// 从上位机发送到接收机      的命令7, 读取状态信息, 包括:8路舵机PWM值和电机转速(rad/s), 
	CMD_APPTCB8 = 0x18,	// 从上位机发送到接收机或手柄的命令8, 校准主板加速度传感器,通知下位机发送原始数据, 发送校准结果给下位机
	
	// Response, Control Board (Receiver) to APP, 接收机,手柄或者控制板反馈给上位机的数据
	RES_APPTCB1 = 0x19,	// 接收机或手柄对APPTCB1的响应, 返回地址和频率
	RES_APPTCB2 = 0x1A,	// 接收机或手柄对APPTCB2的响应, 返回地址写入状态
	RES_APPTCB3 = 0x1B,	// 接收机或手柄对APPTCB3的响应, 返回频率写入状态
	RES_APPTCB4 = 0x1C,	//         手柄对APPTCB4的响应, 无返回
	RES_APPTCB5 = 0x1D,	// 接收机或手柄对APPTCB5的响应, 透传功能, 接收机发送数据给上位机
	RES_APPTCB6 = 0x1E,	// 接收机      对APPTCB6的响应, 返回设置结果
	RES_APPTCB7 = 0x1F,	// 接收机      对APPTCB7的响应, 返回读取状态信息, 包括:8路舵机PWM值和电机转速(rad/s), 
	RES_APPTCB8 = 0x20,	// 接收机或手柄对APPTCB8的响应, 返回原始加速度传感器数据给上位机, 返回校准状态
	
	// Command, APP to Control Board (Receiver), 从上位机发送到接收机,手柄或者控制板
	CMD_APPTCB9  = 0x21,	// 从上位机发送到接收机或手柄的命令, 校准主板磁力计传感器,通知下位机发送原始数据, 发送校准结果给下位机
	CMD_APPTCB10 = 0x22,	// 从上位机发送到接收机或手柄的命令, 校准主板陀螺仪传感器,通知下位机发送原始数据, 发送校准结果给下位机
	CMD_APPTCB11 = 0x23,	// 从上位机发送到接收机或手柄的命令, 
	CMD_APPTCB12 = 0x24,	// 从上位机发送到接收机或手柄的命令, 
	CMD_APPTCB13 = 0x25,	// 从上位机发送到接收机或手柄的命令, 
	CMD_APPTCB14 = 0x26,	// 从上位机发送到接收机      的命令, 
	CMD_APPTCB15 = 0x27,	// 从上位机发送到接收机      的命令, 
	CMD_APPTCB16 = 0x28,	// 从上位机发送到接收机或手柄的命令,
	
	// Response, Control Board (Receiver) to APP, 接收机,手柄或者控制板反馈给上位机的数据
	RES_APPTCB9  = 0x29,	// 接收机或手柄对APPTCB1的响应, 返回原始磁力计传感器数据给上位机, 返回校准状态
	RES_APPTCB10 = 0x2A,	// 接收机或手柄对APPTCB2的响应, 返回原始陀螺仪传感器数据给上位机, 返回校准状态
	RES_APPTCB11 = 0x2B,	// 接收机或手柄对APPTCB3的响应, 
	RES_APPTCB12 = 0x2C,	// 接收机或手柄对APPTCB4的响应, 
	RES_APPTCB13 = 0x2D,	// 接收机或手柄对APPTCB5的响应, 
	RES_APPTCB14 = 0x2E,	// 接收机      对APPTCB6的响应, 
	RES_APPTCB15 = 0x2F,	// 接收机      对APPTCB7的响应, 
	RES_APPTCB16 = 0x30,	// 接收机或手柄对APPTCB8的响应
	
}msgID_e;

/*通讯数据结构 radio message*/
typedef struct
{
	uint16_t head;
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[RADIO_MSG_MAX_SIZE];
	uint16_t checksum;
}radioMsg_t;


void radiolinkInit(void);
bool radiolinkSendPacket(const radioMsg_t *p);
bool radiolinkSendPacketBlocking(const radioMsg_t *p);
bool radiolinkReceivePacket(radioMsg_t *p);
bool radiolinkReceivePacketBlocking(radioMsg_t *p);
void RadioTask(void* param);
uint16_t radioinkFailRxcount(void);
bool radiolinkConnectStatus(void);
void radiolinkEnable(FunctionalState state);


void configParamInit(void);
void configParamTask(void* param);
void writeConfigParamToFlash(void);
void configParamReset(void);

void ParseRadioMsg(radioMsg_t *msg);

#endif /* __RADIO_H */
