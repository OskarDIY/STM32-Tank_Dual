/**
  ******************************************************************************
  * @file    sensor.c
  * @author: Oskar Wei
	* @Mail:   990092230@qq.com
  * @version 1.1
  * @date    2020-05-17
  * @brief   sensor top level api
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT Oskarbot 2020
  ******************************************************************************
  */ 


#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "filter.h"
#include "mpu9250.h"
#include "sensors.h"
#include "ak8963.h"
#include "led.h"
#include "i2c_oled.h"
#include "stm32f10x_it.h"
#include "ekf.h"
#include "radio.h"


/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"



#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_2	
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_2

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

// MPU9250主机模式读取数据 缓冲区长度
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN       	8

typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

BiasObj	gyroBiasRunning;
static Axis3f  gyroBias;

static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static bool isInit = false;
sensorData_t sensors;
state_t state;		/*四轴姿态*/
static int8_t AK8963_ASA[3];

static Axis3i16	gyroRaw;
static Axis3i16	accRaw;
static Axis3i16 magRaw;

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static bool isMPUPresent=false;
static bool isMagPresent=false;

static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN] = {0};

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xSemaphoreHandle sensorsDataReady;


static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);


/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
	
//	if(pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0))
//	{
//		printf("\n(pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0)   success\n");
//	}
//	else
//	{
//		printf("\n(pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0)   failed\n");
//	}
//	
	
	
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

/*传感器中断初始化*/
static void sensorsInterruptInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
		/*使能GPIOD、AFIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	/*使能MPU6500中断*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	portDISABLE_INTERRUPTS();
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line2);
	portENABLE_INTERRUPTS();
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* 传感器器件初始化 */
void sensorsDeviceInit(void)
{
//	int8_t asa[3] = {0};
//	i2cdevInit(I2C2_DEV);
	mpu6500Init(I2C2_DEV);
	vTaskDelay(10);
	
	mpu6500Reset();	// 复位MPU6500
	
	vTaskDelay(20);	// 延时等待寄存器复位
	
	u8 temp = mpu6500GetDeviceID();
	
	// MPU6500 Chip ID: 0x71
	if (temp == 0x71)
	{
		isMPUPresent=true;
		printf("MPU6500 I2C connection [OK].\n");
	}
	else
	{
		printf("MPU6500 I2C connection [FAIL].\n");
	}
	
	mpu6500SetSleepEnabled(false);												// 唤醒MPU6500	
	vTaskDelay(10);		
	mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);				// 设置X轴陀螺作为时钟	
	vTaskDelay(10);																				// 延时等待时钟稳定	
	mpu6500SetTempSensorEnabled(true);										// 使能温度传感器	
	mpu6500SetIntEnabled(false);													// 关闭中断	
	mpu6500SetI2CBypassEnabled(true);											// 旁路模式，磁力计和气压连接到主IIC	
	mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);		// 设置陀螺量程, 2000°/s
	mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);	// 设置加速计量程, 2G
	mpu6500SetAccelDLPF(MPU9250_ACCEL_DLPF_BW_45);				// 设置加速计数字低通滤波

//	mpu6500SetRate(0);// 设置采样速率: 1000 / (1 + 0) = 1000Hz
	mpu6500SetRate(4);																		// 设置采样速率: 1000 / (1 + 4) = 200Hz
	mpu6500SetDLPFMode(MPU9250_GYRO_DLPF_BW_92);					// 设置陀螺数字低通滤波
	
	for (uint8_t i = 0; i < 3; i++)												// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
	
#ifdef SENSORS_ENABLE_MAG_AK8963
	ak8963Init(I2C2_DEV);	//ak8963磁力计初始化
	if (ak8963TestConnection() == true)
	{
		isMagPresent = true;
		ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
//		ak8963GetAdjustment(&asa[0], &asa[1], &asa[2]);
//		AK8963_ASA[0] = (int16_t)asa[0] + 128;
//		AK8963_ASA[1] = (int16_t)asa[1] + 128;
//		AK8963_ASA[2] = (int16_t)asa[2] + 128;
		
		printf("AK8963 I2C connection [OK].\n");
	}
	else
	{
		printf("AK8963 I2C connection [FAIL].\n");
	}
#endif

	/*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
}
/*传感器偏置初始化*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}
/*传感器测试*/
bool sensorsTest(void)
{
	bool testStatus = true;

	if (!isInit)
	{
		printf("Uninitialized\n");
		testStatus = false;
	}
	
	return testStatus;
}

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	u32 i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		Axis3f variance;
		Axis3f mean;

		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}

/* 传感器初始化 */
void sensorsInit(void)
{
	if(isInit) return;

	sensorsDataReady = xSemaphoreCreateBinary();/*创建传感器数据就绪二值信号量*/
	sensorsBiasObjInit(&gyroBiasRunning);
	sensorsDeviceInit();	/*传感器器件初始化*/
	sensorsInterruptInit();	/*传感器中断初始化*/
	
	isInit = true;
}
/*设置传感器从模式读取*/
static void sensorsSetupSlaveRead(void)
{
	mpu6500SetSlave4MasterDelay(9); 	// 从机读取速率: 100Hz = (1000Hz / (1 + 9))

	mpu6500SetI2CBypassEnabled(false);	//主机模式
	mpu6500SetWaitForExternalSensorEnabled(true); 	
	mpu6500SetInterruptMode(0); 		// 中断高电平有效
	mpu6500SetInterruptDrive(0); 		// 推挽输出
	mpu6500SetInterruptLatch(0); 		// 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
	mpu6500SetInterruptLatchClear(1); 	// 中断清除模式(0=status-read-only, 1=any-register-read)
	mpu6500SetSlaveReadWriteTransitionEnabled(false); // 关闭从机读写传输
	mpu6500SetMasterClockSpeed(13); 	// 设置i2c速度400kHz

#ifdef SENSORS_ENABLE_MAG_AK8963
	if (isMagPresent)
	{
		// 设置MPU6500主机要读取的寄存器
		mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); 	// 设置磁力计为0号从机
		mpu6500SetSlaveRegister(0, AK8963_RA_ST1); 				// 从机0需要读取的寄存器
		mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); 	// 读取8个字节(ST1, x, y, z heading, ST2 (overflow check))
		mpu6500SetSlaveDelayEnabled(0, true);
		mpu6500SetSlaveEnabled(0, true);
	}
#endif
	
	mpu6500SetI2CMasterModeEnabled(true);	//使能mpu6500主机模式
	mpu6500SetIntDataReadyEnabled(true);	//数据就绪中断使能
}

/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

/**
 * 根据样本计算重力加速度缩放因子
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
		if (gyroBiasRunning.isBiasValueFound)
		{
			LED_TOGGLE;
		}
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}


/*处理磁力计数据*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) 
	{
		int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
		int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
		int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

//		sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
//		sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
//		sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
		
		
		sensors.mag.x = ((long)headingx * AK8963_ASA[0]) >> 8;
		sensors.mag.y = ((long)headingy * AK8963_ASA[1]) >> 8;
		sensors.mag.z = ((long)headingz * AK8963_ASA[2]) >> 8;
		
		
		magRaw.x = headingx;/*用于上传到上位机*/
		magRaw.y = headingy;
		magRaw.z = headingz;
		
//		magRaw.x = sensors.mag.x;
//		magRaw.y = sensors.mag.y;
//		magRaw.z = sensors.mag.z;
	}
}


/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = (((int16_t) buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	accRaw.x = ax;/*用于上传到上位机*/
	accRaw.y = ay;
	accRaw.z = az;
	gyroRaw.x = gx;
	gyroRaw.y = gy;
	gyroRaw.z = gz;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*计算accScale*/
	}
	
//	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.x =  (gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensors.gyro);

//	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG * accScale;	/*单位 g(9.8m/s^2)*/
	sensors.acc.x =  (ax) * SENSORS_G_PER_LSB_CFG * accScale;	/*单位 g(9.8m/s^2)*/
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG * accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG * accScale;
	applyAxis3fLpf((lpf2pData*)(&accLpf), &sensors.acc);
}



// MPU9250传感器原始数据, 用于上传上位机
int16_t sensor_raw[9] = {0};

// 获取加速计和陀螺仪数据
void Get_AccGyroData(const uint8_t *buffer)
{
	// 注意传感器坐标系位ENU坐标系
	int16_t ax = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ay = (((int16_t) buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gx = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gy = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];
	
	// 用于上传上位机
	sensor_raw[0] = ax;
	sensor_raw[1] = ay;
	sensor_raw[2] = az;
	sensor_raw[3] = gx;
	sensor_raw[4] = gy;
	sensor_raw[5] = gz;
	
	sensors.gyro.x =  (gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	// 单位 °/s
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	
	sensors.acc.x =  (ax) * SENSORS_G_PER_LSB_CFG;
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG;
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG;
}

// 处理磁力计数据
void Get_MagData(const uint8_t *buffer)
{
	// 注意传感器坐标系位NED坐标系
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) 
	{
		int16_t mx = (((int16_t) buffer[2]) << 8) | buffer[1];
		int16_t my = (((int16_t) buffer[4]) << 8) | buffer[3];
		int16_t mz = (((int16_t) buffer[6]) << 8) | buffer[5];
		
		// 用于上传上位机
		sensor_raw[6] = mx;
		sensor_raw[7] = my;
		sensor_raw[8] = mz;
		
		sensors.mag.x = (float)mx;// / MAG_GAUSS_PER_LSB;
		sensors.mag.y = (float)my;// / MAG_GAUSS_PER_LSB;
		sensors.mag.z = (float)mz;// / MAG_GAUSS_PER_LSB;
	}
}


extern Menu menu;
uint8_t ak8963_CaliStage = 0;


/*传感器任务*/
void SensorTask(void *param)
{
	float attitude[3] = {0,0,0};
	uint32_t lastWakeTime = 0, lastPlay = 0;
	uint32_t tick = getSysTickCnt();
	radioMsg_t ackMsg;
	
	float temp_a = 0, temp_b = 0;
	
	sensorsInit();	/*传感器初始化*/
	
	sensorsSetupSlaveRead();/*设置传感器从模式读取*/
	
	menu = START;
	// 等待屏幕播放完开机视频
	while(menu == START)
	{
		vTaskDelay(100);
	}
	
	sensorsAcquire(&sensors, tick);
	
	EKF_AHRSInit((float *)&sensors.acc, (float *)&sensors.mag);
	
	lastWakeTime = tick;

	while (1)
	{
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
			tick = getSysTickCnt();
			
			// 计算确定数据长度
			u8 dataLen = (u8) (SENSORS_MPU6500_BUFF_LEN + (isMagPresent ? SENSORS_MAG_BUFF_LEN : 0));

			i2cdevRead(I2C2_DEV, MPU6500_ADDRESS_AD0_LOW, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
			
			// 处理原始数据
			Get_AccGyroData(&(buffer[0]));
			if (isMagPresent)
			{
				Get_MagData(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			}
			
//			EKF_AHRSUpdate((float *)&sensors.gyro, (float *)&sensors.acc, (float *)&sensors.mag, 0.001f * (float)(tick - lastWakeTime));
//			EKF_AHRSGetAngle(attitude);
			
//			if(attitude[0] >= 0)
//			{
//				attitude[0] = 180 - attitude[0];
//			}
//			else
//			{
//				attitude[0] = -(180 + attitude[0]);
//			}
//			
//			if(attitude[1] <= 0)
//			{
//				attitude[1] = 180 + attitude[1];
//			}
//			else
//			{
//				attitude[1] = attitude[1] - 180;
//			}
			
			state.attitude.roll = attitude[0];
			state.attitude.pitch = attitude[1];
			//state.attitude.yaw = attitude[2];
			
			temp_a = (sensor_raw[6] - 241) / 1.04;
			temp_b = sensor_raw[7] - 72;
			
			if((temp_a >= 0) && (temp_b < 0))
			{
				state.attitude.yaw = 90 + atan(temp_a / temp_b) * 57.2957796f;
			}
			else if((temp_a < 0) && (temp_b <= 0))
			{
				state.attitude.yaw = 90 + atan(temp_a / temp_b) * 57.2957796f;
			}
			else if((temp_a <= 0) && (temp_b > 0))
			{
				state.attitude.yaw = 270 + atan(temp_a / temp_b) * 57.2957796f;
			}
			else if((temp_a > 0) && (temp_b >= 0))
			{
				state.attitude.yaw = 270 + atan(temp_a / temp_b) * 57.2957796f;
			}	
			
			
			lastWakeTime = tick;
				
			LED_TOGGLE;
		}
		
//		if(tick - lastPlay > 50)
//		{	
////			printf("roll: %.2f,  pitch: %.2f,  yaw: %.2f\n" ,state.attitude.roll,state.attitude.pitch, atan((sensors.mag.y-0.2) / 2.5 / (sensors.mag.x-0.8)) * 57.2957796f);
////			printf("A%-4d,%-4d,%-4dB", sensor_raw[6], sensor_raw[7], sensor_raw[8]);
//			printf("yaw:%f\n", atan((float)(sensor_raw[6]-241) / 1.04 / (sensor_raw[7]-72)) * 57.2957796f);

//			lastPlay = tick;
//		}
		
		// 校准磁力计
		// 会有3个处理阶段, 
		// 第一阶段收到上位机读取磁力计数据的指令, 然后持续向上位机发送磁力计数据
		// 第二个阶段, 上位机已经收集到了足够的数据, 发送指令要求下位机停止发送数据
		// 第三个阶段, 上位机已经计算好了校准参数, 这是上位机发送校准参数给下位机, 下位机保存此数据
		// 我们通过状态代码来判断当前校准的阶段, 1:第一阶段, 2:第二阶段, 3:第三阶段
		// 命令格式示例:AA AA 21 len status _ _
		if(ak8963_CaliStage == 1)
		{
			if(tick - lastPlay > 10)
			{
//				printf("A%5.2f,%5.2f,%5.2fB" ,sensors.mag.x, sensors.mag.y, sensors.mag.z);
				ackMsg.head = 0xAAAA;
				ackMsg.msgID = RES_APPTCB9;				// 控制板对APPTCB9的响应, 返回写入状态
				ackMsg.dataLen = 0x18;
				
				ackMsg.data[0] = sensor_raw[0];
				ackMsg.data[1] = sensor_raw[0] >> 8;
				ackMsg.data[2] = sensor_raw[1];
				ackMsg.data[3] = sensor_raw[1] >> 8;
				ackMsg.data[4] = sensor_raw[2];
				ackMsg.data[5] = sensor_raw[2] >> 8;
				
				ackMsg.data[6] = sensor_raw[3];
				ackMsg.data[7] = sensor_raw[3] >> 8;
				ackMsg.data[8] = sensor_raw[4];
				ackMsg.data[9] = sensor_raw[4] >> 8;
				ackMsg.data[10] = sensor_raw[5];
				ackMsg.data[11] = sensor_raw[5] >> 8;
				
				ackMsg.data[12] = sensor_raw[6];
				ackMsg.data[13] = sensor_raw[6] >> 8;
				ackMsg.data[14] = sensor_raw[7];
				ackMsg.data[15] = sensor_raw[7] >> 8;
				ackMsg.data[16] = sensor_raw[8];
				ackMsg.data[17] = sensor_raw[8] >> 8;
				
				// CRC16校验和
				ackMsg.checksum = CRC_Table((uint8_t *)&ackMsg, ackMsg.dataLen - 2);
				
				// 发送数据
				for(int i = 0; i < ackMsg.dataLen - 2; i++)
				{
					while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
					{
						
					}
					USART_SendData(USART1, ((uint8_t *)&ackMsg)[i]);
				}
				
				// 因为STM32单片机是小端模式, 所以校验值的低字节存储在低地址, 先发送校验的低字节
				while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
				{

				}
				USART_SendData(USART1, ackMsg.checksum);
				
				// 发送校验的高字节
				while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET)
				{

				}
				USART_SendData(USART1, ackMsg.checksum >> 8);
				
				lastPlay = tick;
			}
		}
		else if(ak8963_CaliStage == 2)
		{
			
		}
		else if(ak8963_CaliStage == 3)
		{
			
		}
		
		
		
		
		
		
	}
}











/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)	
{	
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
}


/*二阶低通滤波*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (uint8_t i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
/*传感器数据校准*/
bool sensorsAreCalibrated()	
{
	return gyroBiasFound;
}
/*上位机获取读取原始数据*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag)
{
	*acc = accRaw;
	*gyro = gyroRaw;
	*mag = magRaw;
}

bool getIsMPU9250Present(void)
{
	bool value = isMPUPresent;
#ifdef SENSORS_ENABLE_MAG_AK8963
	value &= isMagPresent;
#endif
	return value;
}


void EXTI2_IRQHandler(void)
{
	// sensor
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
		
		
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

		if (xHigherPriorityTaskWoken)
		{
			portYIELD();
		}
		
	}
}
