/**
  ******************************************************************************
  * @file    flash.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-08-23
  * @brief   embedded flash
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
	* 该程序仅供学习使用，未经作者允许，不得用于其它任何用途
  ******************************************************************************
  */


#include "flash.h"
#include "radio.h"
#include "key.h"
#include "utils.h"
#include <stdio.h>

// 系统设置
extern systemConfig_t configParam;
extern systemConfig_t configParamDefault;


/**
  * @brief  Load configurations from flash memory.
  * @param  None
  * @retval bool
  */
void LoadConfigData(void)
{
	uint32_t address = 0x00;				//记录写入的地址
	uint32_t data = 0x0;						//记录写入的数据
	uint8_t i= 0;
	uint16_t checkSum = 0;
	
	uint8_t config[CONFIG_DATA_SIZE] = {0};
	
	// 读取Flash数据
  address = WRITE_START_ADDR;
	
	for(i = 0; i < CONFIG_DATA_SIZE; i += 4)
	{
		data = *(__IO uint32_t*) address;
		config[i]			= data;
		config[i + 1] = data >> 8;
		config[i + 2] = data >> 16;
		config[i + 3] = data >> 24;
		
		address += 4;
	}
	
	checkSum = CRC_Table(config, CONFIG_DATA_SIZE - 2);
	
	if((config[10] != (checkSum & 0xFF)) || (config[11] != (checkSum >> 8)))
	{
		printf("Error: Loading config data from flash failed!\nLoading default parameters and saving them to flash!");
		configParam = configParamDefault;
		SaveConfigData();
	}
	else
	{
		// 无线地址
		configParam.radio.addr[0] = config[0];
		configParam.radio.addr[1] = config[1];
		configParam.radio.addr[2] = config[2];
		configParam.radio.addr[3] = config[3];
		configParam.radio.addr[4] = config[4];
		
		// 无线频率
		configParam.radio.channel = config[5];
		
		configParam.radio.dataRate = RADIO_DATARATE;
		
		// 文件系统标志
		configParam.fs_mark = config[6];
		configParam.fs_mark = config[7] << 8;
		configParam.fs_mark = config[8] << 16;
		configParam.fs_mark = config[9] << 24;
		
	}
	
}


/**
  * @brief  Save configurations to flash memory.
  * @param  None
  * @retval bool
  */
bool SaveConfigData(void)
{
	uint32_t address = 0x00;				//记录写入的地址
	uint32_t data = 0x0;						//记录写入的数据
	uint8_t i= 0;
	uint16_t checkSum = 0;
	
	FLASH_Status FLASHStatus = FLASH_COMPLETE;	//记录每次擦除的结果	
	bool programStatus = true;									//记录整个测试结果
	
	
	uint8_t config[CONFIG_DATA_SIZE] = {0};
	
	// 无线地址
	config[0] = configParam.radio.addr[0];
	config[1] = configParam.radio.addr[1];
	config[2] = configParam.radio.addr[2];
	config[3] = configParam.radio.addr[3];
	config[4] = configParam.radio.addr[4];
	
	// 无线频率
	config[5] = configParam.radio.channel;
	
	// 文件系统标志
	config[6] = configParam.fs_mark;
	config[7] = configParam.fs_mark >> 8;
	config[8] = configParam.fs_mark >> 16;
	config[9] = configParam.fs_mark >> 24;
	
	checkSum = CRC_Table(config, CONFIG_DATA_SIZE - 2);
	config[10] = checkSum;
	config[11] = checkSum >> 8;
	

  /* 解锁 */
  FLASH_Unlock();

  /* 清空所有标志位 */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /* 按页擦除*/
	FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR);
  
  /* 向内部FLASH写入数据 */
  address = WRITE_START_ADDR;
	i = 0;

  while((i < CONFIG_DATA_SIZE) && (FLASHStatus == FLASH_COMPLETE))
  {
		data = 0;
		data = config[i] | (config[i + 1] << 8) | (config[i + 2] << 16) | (config[i + 3] << 24);
    FLASHStatus = FLASH_ProgramWord(address, data);
    address += 4;
		i += 4;
  }

  FLASH_Lock();
  
  /* 检查写入的数据是否正确 */
  address = WRITE_START_ADDR;
	i = 0;

  while((i < CONFIG_DATA_SIZE) && (programStatus != false))
  {
		data = config[i] | (config[i + 1] << 8) | (config[i + 2] << 16) | (config[i + 3] << 24);
		
    if((*(__IO uint32_t*) address) != data)
    {
      programStatus = false;
    }
    address += 4;
		i += 4;
  }
	
	return programStatus;
	
}
