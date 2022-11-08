/**
  ******************************************************************************
  * @file    utils.c
  * @author: Oskar Wei
	* @Mail:   990092230@qq.com
  * @version 1.1
  * @date    2020-05-17
  * @brief   utils
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT Oskarbot 2020
  ******************************************************************************
  */ 

#include "led.h"
#include "utils.h"
#include "spi_flash.h"
#include "string.h"
#include "i2c_oled.h"
#include "usart1.h"
#include "flash.h"

#include "FreeRTOS.h"


FATFS fs;													/* FatFs文件系统对象 */
FIL fnew;													/* 文件对象 */
FRESULT res_flash;                /* 文件操作结果 */
UINT fnum;            					  /* 文件成功读写数量 */
BYTE ReadBuffer[2048]={0};        /* 读缓冲区 */
BYTE WriteBuffer[] =              /* 写缓冲区*/
"\nLove your neighbor as yourself.\r\n"; 



BYTE WriteBuffer2[] =              /* 写缓冲区*/
"\nHow was that ?\r\n"; 

char fpath[100];                  /* 保存当前扫描路径 */

// 擦除整片FLASH芯片
void Erase_SPI_Flash(void)
{
	SPI_FLASH_Init();
	SPI_FLASH_BulkErase();
	printf(">>FLASH擦除完成...\r\n");
}


// 系统设置
extern systemConfig_t configParam;

// 文件系统检测
void FileSystem_Init(void)
{
	FATFS *pfs;
	DWORD fre_clust, fre_sect, tot_sect;
	
	//在外部SPI Flash挂载文件系统,文件系统挂载时会对SPI设备初始化
	//初始化函数调用流程如下
	//f_mount()->find_volume()->disk_initialize->SPI_FLASH_Init()
	res_flash = f_mount(&fs,"1:",1);
	
	/*----------------------- 格式化测试 -----------------*/  
	/* 如果没有文件系统就格式化创建创建文件系统 */
	if(res_flash == FR_NO_FILESYSTEM)
	{
		if(configParam.fs_mark == FS_MARK)
		{
			__set_FAULTMASK(1); // 关闭总中断
			NVIC_SystemReset(); // 请求复位重启
		}
		
		printf(">>SPI FLASH中未发现文件系统,即将进行格式化...\r\n");
    /* 格式化 */
		res_flash=f_mkfs("1:",0,0);							
		
		if(res_flash == FR_OK)
		{
			printf(">>SPI FLASH格式化完成...\r\n");
      /* 格式化后,先取消挂载 */
			res_flash = f_mount(NULL,"1:",1);			
      /* 重新挂载	*/			
			res_flash = f_mount(&fs,"1:",1);
		}
		else
		{
			LED_TOGGLE;
			printf("!!SPI FLASH格式化失败...\r\n");
			while(1);
		}
	}
  else if(res_flash != FR_OK)
  {
    printf("!!SPI FLASH文件系统挂载失败...(%d)\r\n",res_flash);
    printf("!!可能原因,SPI FLASH初始化未成功...\r\n");
		while(1);
  }
  else
  {
		if(configParam.fs_mark != FS_MARK)
		{
			SaveConfigData();
		}
		
    printf(">>SPI FLASH文件系统挂载成功...\r\n");
  }
	
	
	
	printf("\n*************** 设备信息获取 ***************\r\n");
  /* 获取设备信息和空簇大小 */
  res_flash = f_getfree("1:", &fre_clust, &pfs);

  /* 计算得到总的扇区个数和空扇区个数 */
  tot_sect = (pfs->n_fatent - 2) * pfs->csize;
  fre_sect = fre_clust * pfs->csize;

  /* 打印信息(4096 字节/扇区) */
  printf("\n>>设备总空间：%10lu KB.\n>>可用空间：  %10lu KB.\n", tot_sect *4, fre_sect *4);
	
	printf("****************文件扫描测试***************\r\n");
  strcpy(fpath,"1:");
  scan_files(fpath);
	
}



/**
  * @brief  scan_files 递归扫描FatFs内的文件
  * @param  path:初始扫描路径
  * @retval result:文件系统的返回值
  */
FRESULT scan_files (char* path) 
{ 
  FRESULT res; 		//部分在递归过程被修改的变量,不用全局变量	
  FILINFO fno; 
  DIR dir; 
  int i;            
  char *fn;        // 文件名	
	
#if _USE_LFN 
  /* 长文件名支持 */
  /* 简体中文需要2个字节保存一个“字”*/
  static char lfn[_MAX_LFN*2 + 1]; 	
  fno.lfname = lfn; 
  fno.lfsize = sizeof(lfn); 
#endif 
  //打开目录
  res = f_opendir(&dir, path); 
  if (res == FR_OK) 
	{ 
    i = strlen(path); 
    for (;;) 
		{ 
      //读取目录下的内容,再读会自动读下一个文件
      res = f_readdir(&dir, &fno); 								
      //为空时表示所有项目读取完毕,跳出
      if (res != FR_OK || fno.fname[0] == 0) break; 	
#if _USE_LFN 
      fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
      fn = fno.fname; 
#endif 
      //点表示当前目录,跳过			
      if (*fn == '.') continue; 	
      //目录,递归读取      
      if (fno.fattrib & AM_DIR)         
			{ 			
        //合成完整目录名        
        sprintf(&path[i], "/%s", fn); 		
        //递归遍历         
        res = scan_files(path);	
        path[i] = 0;         
        //打开失败,跳出循环        
        if (res != FR_OK) 
					break; 
      } 
			else 
			{ 
				printf("%s/%s\r\n", path, fn);								//输出文件名	
        /* 可以在这里提取特定格式的文件路径 */        
      }//else
    } //for
  } 
  return res; 
}



void FileSystem_Test(void)
{
  
/*----------------------- 文件系统测试:写测试 -------------------*/
	/* 打开文件,每次都以新建的形式打开,属性为可写 */
	printf("\r\n******即将进行文件写入测试...******\r\n");	
	res_flash = f_open(&fnew, "1:testfile.txt",FA_CREATE_ALWAYS|FA_WRITE|FA_READ);
	if ( res_flash == FR_OK )
	{
		printf(">>打开/创建testfile.txt文件成功\r\n");
    /* 将指定存储区内容写入到文件内 */
//		res_flash=f_write(&fnew,WriteBuffer,sizeof(WriteBuffer),&fnum);
		
		f_printf(&fnew,"%s",WriteBuffer);
    if(res_flash==FR_OK)
    {
      printf(">>文件写入成功,写入字节数据:%d\n",fnum);
      printf(">>向文件写入的数据为:\r\n%s\r\n",WriteBuffer);
    }
    else
    {
      printf("!!文件写入失败:(%d)\n",res_flash);
    }
		
		/* 不再读写,关闭文件 */
    f_close(&fnew);
	}
	else
	{	
		LED_TOGGLE;
		printf("!!打开/创建文件失败...\r\n");
	}
	
	res_flash = f_open(&fnew, "1:testfile.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	if ( res_flash == FR_OK )
	{
		printf(">>打开testfile.txt文件成功\r\n");
		f_lseek(&fnew,f_size(&fnew));
		
    /* 将指定存储区内容写入到文件内 */
//		res_flash=f_write(&fnew,WriteBuffer2,sizeof(WriteBuffer2),&fnum);
		f_printf(&fnew,"%s",WriteBuffer2);
		
    if(res_flash==FR_OK)
    {
      printf(">>文件写入成功,写入字节数据:%d\n",fnum);
      printf(">>向文件写入的数据为:\r\n%s\r\n",WriteBuffer2);
    }
    else
    {
      printf("!!文件写入失败:(%d)\n",res_flash);
    }    
		/* 不再读写,关闭文件 */
    f_close(&fnew);
	}
	else
	{	
		LED_TOGGLE;
		printf("!!打开/创建文件失败...\r\n");
	}
	
/*------------------- 文件系统测试:读测试 --------------------------*/
	printf("****** 即将进行文件读取测试... ******\r\n");
	res_flash = f_open(&fnew, "1:testfile.txt",FA_OPEN_EXISTING | FA_READ); 	 
	if(res_flash == FR_OK)
	{
		LED_TOGGLE;
		printf(">>打开文件成功.\r\n");
		res_flash = f_read(&fnew, ReadBuffer, f_size(&fnew), &fnum); 
    if(res_flash==FR_OK)
    {
      printf(">>文件读取成功,读到字节数据:%d\r\n",fnum);
      printf(">>读取得的文件数据为:\r\n%s\r\n", ReadBuffer);	
    }
    else
    {
      printf("!!文件读取失败:(%d)\n",res_flash);
    }		
	}
	else
	{
		LED_TOGGLE;
		printf("!!打开文件失败...\r\n");
	}
	/* 不再读写,关闭文件 */
	f_close(&fnew);	
	
//	File_Creater("animation.bin");
	
//	Show_Animation();
}














#define MAGIC_ASSERT_INDICATOR 0x2f8a001f

typedef struct SNAPSHOT_DATA 
{
	u32 magicNumber;
	char* fileName;
	int line;
} SNAPSHOT_DATA;

// The .nzds section is not cleared at startup, data here will survive a
// reset (by the watch dog for instance)
SNAPSHOT_DATA snapshot __attribute__((section(".nzds"))) = 
{
	.magicNumber = 0,
	.fileName = "",
	.line = 0
};


void storeAssertSnapshotData(char *file, int line)
{
	snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
	snapshot.fileName = file;
	snapshot.line = line;
}

void assertFail(char *exp, char *file, int line)
{
	portDISABLE_INTERRUPTS();
	storeAssertSnapshotData(file, line);
	printf("Assert failed %s:%d\n", file, line);

	while (1);
}


void printAssertSnapshotData()
{
	if (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) 
	{
		printf("Assert failed at %s:%d\n", snapshot.fileName, snapshot.line);
	} else 
	{
		printf("No assert information found\n");
	}
}





/* Table of CRC values for high-order byte */
const uint8_t crcTableHigh[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};
/* Table of CRC values for low-order byte */
const uint8_t crcTableLow[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

// Modbus CRC16, 宽度: 16, 公式:x16+x15+x2+x1, Hex: 8005, 初始值: FFFF, XOROUT: 0000
uint16_t CRC_Table(uint8_t *ptr, uint16_t len)
{
	uint8_t crchi = 0xff;
	uint8_t crclo = 0xff; 
	uint16_t index;
	while(len--) 
	{
		index = crclo ^ *ptr++; 
		crclo = crchi ^ crcTableHigh[index];
		crchi = crcTableLow[index];
	}
	return (crchi << 8 | crclo);
}





