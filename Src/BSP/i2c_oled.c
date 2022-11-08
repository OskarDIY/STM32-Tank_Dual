 /**
  ******************************************************************************
  * @file    i2c_oled.c
  * @author  Oskar Wei
  * @version V1.0
  * @date    2020-05-02
  * @brief   i2c oled low level init
  ******************************************************************************
  * @attention
  *
  * Mail: 990092230@qq.com
  * Shop: www.mindsilicon.com
  *
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "i2c_oled.h"
#include "led.h"
#include "delay.h"
#include "oledfont.h"
#include "i2c_device.h"
#include "adc.h"
#include "stm32f10x_it.h"
#include "sensors.h"
#include "motor.h"

extern float ADC_ConvertedValueLocal[NOFCHANEL];
extern state_t state;
extern uint16_t ultrasonic_distance1;
extern uint16_t ultrasonic_distance2;
extern motorStatus_t motorStatus;

void I2C2_Init(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	/*STM32F103RCT6芯片的硬件I2C: PB10 -- SCL; PB11 -- SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	// I2C必须开漏输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// 使用I2C2
	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
	// 主机的I2C地址,可以自己设置
	I2C_InitStructure.I2C_OwnAddress1 = 0x01;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	// 频率400K
	I2C_InitStructure.I2C_ClockSpeed = 400000;

	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
}

void I2C2_WriteByte(uint8_t addr,uint8_t data)
{
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	
	// 开启I2C1
	I2C_GenerateSTART(I2C2, ENABLE);
	// 主模式
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	
	// 器件地址 -- 默认0x78
	I2C_Send7bitAddress(I2C2, OLED_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	// 寄存器地址
	I2C_SendData(I2C2, addr);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	// 发送数据
	I2C_SendData(I2C2, data);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	// 关闭I2C2总线
	I2C_GenerateSTOP(I2C2, ENABLE);
}


// 写命令
void OLED_WriteCmd(unsigned char I2C_Command)
{
//	I2C2_WriteByte(0x00, I2C_Command);
//		printf("\nCMD: %x\n", I2C_Command);
	
	i2cdevWriteByte(I2C2_DEV, OLED_ADDRESS >> 1, 0x00, I2C_Command);
}

// 写数据
void OLED_WriteData(unsigned char I2C_Data)
{
//	I2C2_WriteByte(0x40, I2C_Data);
	i2cdevWriteByte(I2C2_DEV, OLED_ADDRESS >> 1, 0x40, I2C_Data);
}

void OLED_Init(void)
{
	// 延时不可去掉
	delay_ms(500);
	delay_ms(500);
	
	OLED_WriteCmd(0xAE); //display off
	OLED_WriteCmd(0x20);	//Set Memory Addressing Mode	
	OLED_WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	OLED_WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	OLED_WriteCmd(0xc8);	//Set COM Output Scan Direction
	OLED_WriteCmd(0x00); //---set low column address
	OLED_WriteCmd(0x10); //---set high column address
	OLED_WriteCmd(0x40); //--set start line address
	OLED_WriteCmd(0x81); //--set contrast control register
	OLED_WriteCmd(0xff); //亮度调节 0x00~0xff
	OLED_WriteCmd(0xa1); //--set segment re-map 0 to 127
	OLED_WriteCmd(0xa6); //--set normal display
	OLED_WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	OLED_WriteCmd(0x3F); //
	OLED_WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	OLED_WriteCmd(0xd3); //-set display offset
	OLED_WriteCmd(0x00); //-not offset
	OLED_WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	OLED_WriteCmd(0xf0); //--set divide ratio
	OLED_WriteCmd(0xd9); //--set pre-charge period
	OLED_WriteCmd(0x22); //
	OLED_WriteCmd(0xda); //--set com pins hardware configuration
	OLED_WriteCmd(0x12);
	OLED_WriteCmd(0xdb); //--set vcomh
	OLED_WriteCmd(0x20); //0x20,0.77xVcc
	OLED_WriteCmd(0x8d); //--set DC-DC enable
	OLED_WriteCmd(0x14); //
	OLED_WriteCmd(0xaf); //--turn on oled panel
}

void OLED_Reset(void)
{
	OLED_WriteCmd(0xAE); //display off
	OLED_WriteCmd(0x20);	//Set Memory Addressing Mode	
	OLED_WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	OLED_WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	OLED_WriteCmd(0xc8);	//Set COM Output Scan Direction
	OLED_WriteCmd(0x00); //---set low column address
	OLED_WriteCmd(0x10); //---set high column address
	OLED_WriteCmd(0x40); //--set start line address
	OLED_WriteCmd(0x81); //--set contrast control register
	OLED_WriteCmd(0xff); //亮度调节 0x00~0xff
	OLED_WriteCmd(0xa1); //--set segment re-map 0 to 127
	OLED_WriteCmd(0xa6); //--set normal display
	OLED_WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	OLED_WriteCmd(0x3F); //
	OLED_WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	OLED_WriteCmd(0xd3); //-set display offset
	OLED_WriteCmd(0x00); //-not offset
	OLED_WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	OLED_WriteCmd(0xf0); //--set divide ratio
	OLED_WriteCmd(0xd9); //--set pre-charge period
	OLED_WriteCmd(0x22); //
	OLED_WriteCmd(0xda); //--set com pins hardware configuration
	OLED_WriteCmd(0x12);
	OLED_WriteCmd(0xdb); //--set vcomh
	OLED_WriteCmd(0x20); //0x20,0.77xVcc
	OLED_WriteCmd(0x8d); //--set DC-DC enable
	OLED_WriteCmd(0x14); //
	OLED_WriteCmd(0xaf); //--turn on oled panel
}

// 设置起始点坐标
void OLED_SetPos(unsigned char x, unsigned char y)
{
	OLED_WriteCmd(0xb0+y);
	OLED_WriteCmd(((x&0xf0)>>4)|0x10);
	OLED_WriteCmd((x&0x0f));
}

// 全屏填充
void OLED_Fill(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WriteCmd(0xb0+m);	//page0-page1
		OLED_WriteCmd(0x00);		//low column start address
		OLED_WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
		{
			OLED_WriteData(fill_Data);
		}
	}
}

void OLED_CLS(void)//清屏
{
	uint8_t m, n[128] = {0};
	for(m=0;m<8;m++)
	{
		OLED_WriteCmd(0xb0+m);	//page0-page1
		OLED_WriteCmd(0x00);		//low column start address
		OLED_WriteCmd(0x10);		//high column start address
		
		i2cdevWrite(I2C2_DEV, OLED_ADDRESS >> 1, 0x40, 128, n);
	}
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          : 
// Parameters     : none
// Description    : 将OLED从休眠中唤醒
//--------------------------------------------------------------
void OLED_ON(void)
{
	OLED_WriteCmd(0X8D);  //设置电荷泵
	OLED_WriteCmd(0X14);  //开启电荷泵
	OLED_WriteCmd(0XAF);  //OLED唤醒
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          : 
// Parameters     : none
// Description    : 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
	OLED_WriteCmd(0X8D);  //设置电荷泵
	OLED_WriteCmd(0X10);  //关闭电荷泵
	OLED_WriteCmd(0XAE);  //OLED休眠
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); ch[] -- 要显示的字符串; TextSize -- 字符大小(1:6*8 ; 2:8*16)
// Description    : 显示codetab.h中的ASCII字符,有6*8和8*16可选择
//--------------------------------------------------------------
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				
//				for(i=0;i<6;i++)
//					OLED_WriteData(F6x8[c][i]);
				i2cdevWrite(I2C2_DEV, OLED_ADDRESS >> 1, 0x40, 6, (uint8_t*)&(F6x8[c]));
				
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
//				for(i=0;i<8;i++)
//					OLED_WriteData(F8X16[c*16+i]);
				i2cdevWrite(I2C2_DEV, OLED_ADDRESS >> 1, 0x40, 8, (uint8_t*)&(F8X16[c*16]));
				
				OLED_SetPos(x,y+1);
//				for(i=0;i<8;i++)
//					OLED_WriteData(F8X16[c*16+i+8]);
				i2cdevWrite(I2C2_DEV, OLED_ADDRESS >> 1, 0x40, 8, (uint8_t*)&(F8X16[c*16+8]));
				x += 8;
				j++;
			}
		}break;
	}
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); N:汉字在oledfont.h中的索引
// Description    : 显示codetab.h中的汉字,16*16点阵
//--------------------------------------------------------------
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned char wm=0;
	unsigned int  adder=32*N;
	OLED_SetPos(x , y);
	for(wm = 0;wm < 16;wm++)
	{
		OLED_WriteData(F16x16[adder]);
		adder += 1;
	}
	OLED_SetPos(x,y + 1);
	for(wm = 0;wm < 16;wm++)
	{
		OLED_WriteData(F16x16[adder]);
		adder += 1;
	}
}




//--------------------------------------------------------------
// Prototype      : void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
// Calls          : 
// Parameters     : x0,y0 -- 起始点坐标(x0:0~127, y0:0~7); x1,y1 -- 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
// Description    : 显示BMP位图
//--------------------------------------------------------------
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char y;
	
	j=0;
	
  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
		
		i2cdevWrite(I2C2_DEV, OLED_ADDRESS >> 1, 0x40, 128, &BMP[j]);
		
		j = j + 128;
//    for(x=x0;x<x1;x++)
//		{
//			OLED_WriteData(BMP[j++]);
//		}
	}
}


// res文件中165,274为分界线
// 调用此方法前请确认文件系统已经成功加载。
void PlayAnimation(void)
{
	FIL fnew;													/* 文件对象 */
	FRESULT res_flash;                /* 文件操作结果 */
	UINT fnum;            					  /* 文件成功读写数量 */
	int i = 0;
	unsigned char bmp[128*8];
	
	OLED_Init();
	
	while(1)
	{
		printf(">>PlayAnimation Task...\r\n");
		
		res_flash = f_open(&fnew, "1:/res.bin",FA_OPEN_EXISTING | FA_READ); 
	
		if ( res_flash == FR_OK )
		{
			printf(">>开机动画加载成功...\r\n");
			
//			i = 75;
			i = 6400;
			while(i--)
			{
				f_read(&fnew, bmp, 128*8, &fnum);
//				xQueueSendToBack(animationQueue, &bmp, 100);
				OLED_DrawBMP(0,0,128,8,(unsigned char *)bmp);
//				printf("PlayAnimation task running\n");
			}
		}
		else
		{	
			LED_TOGGLE;
			printf("!!未发现开机动画....\r\n");
		}
		
		/* 不再读写,关闭文件 */
		f_close(&fnew);	
	}
}



// 因为MPU9250和OLED屏幕挂在同一个总线下,开机时先给MPU9250初始化,之后再用于其他任务
Menu menu = MPU_INIT;

/**
  * @brief  菜单任务,用于控制屏幕显示
  * @param  无
  * @retval 无
  * 调用此方法前请确认文件系统已经成功加载外置flash芯片中的res资源文件
  * res文件中165帧之前为开机动画,166至274为关机动画,
  */
void MenuTask(void *param)
{
	FIL fnew;													/* 文件对象 */
	FRESULT res_flash;                /* 文件操作结果 */
	UINT fnum;            					  /* 文件成功读写数量 */
	int i = 0;
	unsigned char bmp[128*8];
	
	// 等待MPU初始化完成并释放总线
	while(menu == MPU_INIT)
	{
		vTaskDelay(50);
	}
	
	// 初始化OLED屏幕
	OLED_Init();
	
	while(1)
	{
//		printf(">>menuTask...\r\n");
		
		switch(menu)
		{
			case START:
				// 读取资源文件
				res_flash = f_open(&fnew, "1:/res.bin",FA_OPEN_EXISTING | FA_READ); 
				f_lseek(&fnew, 160*1024);
			
				if ( res_flash == FR_OK )
				{
					printf(">>动画资源加载成功...\r\n");
					
					// 资源文件res中，0到273帧为开机动画
					i = 273;
					while(i--)
					{
						f_read(&fnew, bmp, 128*8, &fnum);
						OLED_DrawBMP(0,0,128,8,(unsigned char *)bmp);
					}
				}
				else
				{	
					LED_TOGGLE;
					printf("!!未发现动画资源....\r\n");
				}
				
				/* 不再读写,关闭文件 */
				f_close(&fnew);	
				OLED_CLS();
				menu = RUNNING;
				break;
			
			case SHUTDOWN:
				res_flash = f_open(&fnew, "1:/res.bin",FA_OPEN_EXISTING | FA_READ); 
							
				if ( res_flash == FR_OK )
				{
					printf(">>动画资源加载成功...\r\n");
					
					i = 160;
					while(i--)
					{
						f_read(&fnew, bmp, 128*8, &fnum);
						OLED_DrawBMP(0,0,128,8,(unsigned char *)bmp);
					}
				}
				else
				{	
					LED_TOGGLE;
					printf("!!未发现动画资源....\r\n");
				}

				/* 不再读写,关闭文件 */
				f_close(&fnew);
				menu = NOTDRAW;
				break;
				;
				
			case LOWBAT:
				res_flash = f_open(&fnew, "1:/res.bin",FA_OPEN_EXISTING | FA_READ); 
				f_lseek(&fnew, 434*1024);
							
				if ( res_flash == FR_OK )
				{
					printf(">>动画资源加载成功...\r\n");
					
					i = 1;
					while(i--)
					{
						f_read(&fnew, bmp, 128*8, &fnum);
						OLED_DrawBMP(0,0,128,8,(unsigned char *)bmp);
					}
				}
				else
				{	
					LED_TOGGLE;
					printf("!!未发现动画资源....\r\n");
				}

				/* 不再读写,关闭文件 */
				f_close(&fnew);
				menu = NOTDRAW;
				break;
				
			case NOTDRAW:
				vTaskDelay(100);
				break;
			
			case RUNNING:
				OLED_ShowParm();
				vTaskDelay(50);
				break;
			
			default:
				break;
		}
	}
}

/**
  * @brief  显示系统参数 
  * @param  无
  * @retval 无
  */
void OLED_ShowParm(void)
{
	char volt1[20], volt2[20], distance1[20], distance2[20], motor1[20], motor2[20];
//	
	sprintf(volt2, "Main  Voltage:%5.2f", ADC_ConvertedValueLocal[1]);
	OLED_ShowStr(0, 0, (uint8_t*)(&volt2), 1);
	
	sprintf(volt1, "Servo Voltage:%5.2f", ADC_ConvertedValueLocal[0]);
	OLED_ShowStr(0, 1, (uint8_t*)(&volt1), 1);
	
	sprintf(distance1, "distance1:%5dmm", ultrasonic_distance1);
	OLED_ShowStr(0, 2, (uint8_t*)(&distance1), 1);
	
	sprintf(distance2, "distance2:%5dmm", ultrasonic_distance2);
	OLED_ShowStr(0, 3, (uint8_t*)(&distance2), 1);
	
	sprintf(motor1, "motor1_pwm:%5d", motorStatus.motor1_pwm);
	OLED_ShowStr(0, 4, (uint8_t*)(&motor1), 1);
	
	sprintf(motor2, "motor2_pwm:%5d", motorStatus.motor2_pwm);
	OLED_ShowStr(0, 5, (uint8_t*)(&motor2), 1);
	
	
//	
//	vTaskDelay(2000);
//	OLED_CLS();
//	
//	OLED_ShowStr(0, 0,(uint8_t*)"Welcome to visit our", 1);
//	OLED_ShowStr(0, 1,(uint8_t*)"website!", 1);
//	OLED_ShowStr(0, 3,(uint8_t*)"www.mindsilicon.com", 1);
//	
//	vTaskDelay(2000);
//	OLED_CLS();
	
	
//	sprintf(volt1, "Yaw: %5.2f", state.attitude.yaw);
//	OLED_ShowStr(0, 1, (uint8_t*)(&volt1), 1);
	
	vTaskDelay(50);
//	OLED_CLS();
	
}
