/**
  ******************************************************************************
  * @file    mpu9250.c
  * @author  yzyan
  * @version V1.3
  * @date    2018-1-9
  * @brief   This file provides all the mpu9250 firmware functions.
  ******************************************************************************
**/


#include"mpu9250.h"
#include "delay.h"
#include<stdio.h>
#include "FastMath.h"
#include "math.h"
#include "EpRomData.h"
#include "usart.h"
#include "IOI2C.h"

#define I2C_SPEED			300000 //no more than 400K
#define I2C_SLAVE_ADDR 		0XA0
#define Timed(x) Timeout = 0xFFFFF; while (x) { if (Timeout-- == 0) goto errReturn;}

//float Acc_Scale;
//float GYRO_Scale;
//float MAG_Scale;
float ACC_BIAS[3],GYRO_BIAS[3];

int16_t  MPU6050_FIFO[6][11];
//int16_t AK8963_FIFO[3][5];
static int8_t MPU6050_FIFO_PTR=0;
//static int8_t AK8963_FIFO_PTR=0;

int16_t AxOffset,AyOffset,AzOffset;
int16_t GxOffset,GyOffset,GzOffset;
float MxOffset,MyOffset,MzOffset;
float Mag_ASA[3];
float AxScale,AyScale,AzScale;
float GxScale,GyScale,GzScale;
float MxScale,MyScale,MzScale;
float lastUpdate;
IMU_config_info IMUconfig;

int16_t MxMax,MyMax,MzMax;
int16_t MxMin,MyMin,MzMin;
/**
 *  Names of events used in stdperipheral library
 *
 *      I2C_EVENT_MASTER_MODE_SELECT                          : EV5
 *      I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6     
 *      I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
 *      I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
 *     
 **/

/*
static bool IMU_I2C_Read(I2C_TypeDef* I2Cx,uint8_t SlaveAddress,uint8_t RegAddress,uint8_t *buf,uint32_t bufSize)
{
	__IO uint32_t Timeout = 0;

	//    I2Cx->CR2 |= I2C_IT_ERR;  interrupts for errors 
	if (!bufSize)return Success;

	// Wait for idle I2C interface
	Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	// Enable Acknowledgement, clear POS flag
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
	// Intiate Start Sequence (wait for EV5
	I2C_GenerateSTART(I2Cx, ENABLE);
	Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// Send Address
	I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Transmitter);
	// EV6
	Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	// send register address
	I2C_SendData(I2C1,RegAddress);
	Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));  
	
	  // Intiate Start Sequence (wait for EV5
	I2C_GenerateSTART(I2Cx, ENABLE);
	Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// Send Address
	I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Receiver);
	// EV6
	Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR));

	if (bufSize == 1)
	{
		// Clear Ack bit      
		I2C_AcknowledgeConfig(I2Cx, DISABLE);       
		// EV6_1 -- must be atomic -- Clear ADDR, generate STOP
		__disable_irq();
		(void) I2Cx->SR2;              
		I2C_GenerateSTOP(I2Cx,ENABLE);
		__enable_irq();
		// Receive data   EV7
		Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE));
		*buf++ = I2C_ReceiveData(I2Cx);
	}
	else if (bufSize == 2)
	{
		// Set POS flag
		I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);

		// EV6_1 -- must be atomic and in this order
		__disable_irq();
		(void) I2Cx->SR2;                           // Clear ADDR flag
		I2C_AcknowledgeConfig(I2Cx, DISABLE);       // Clear Ack bit
		__enable_irq();

		// EV7_3  -- Wait for BTF, program stop, read data twice
		Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));

		__disable_irq();
		I2C_GenerateSTOP(I2Cx,ENABLE);
		*buf++ = I2Cx->DR;
		__enable_irq();

		*buf++ = I2Cx->DR;
	}
	else 
	{
		(void) I2Cx->SR2;                           // Clear ADDR flag
		while (bufSize-- != 3)
		{
			// EV7 -- cannot guarantee 1 transfer completion time, wait for BTF 						//        instead of RXNE
			Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)); 
			*buf++ = I2C_ReceiveData(I2Cx);
		}

		Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)); 
		// EV7_2 -- Figure 1 has an error, doesn't read N-2 !
		I2C_AcknowledgeConfig(I2Cx, DISABLE);           // clear ack bit
		__disable_irq();
		*buf++ = I2C_ReceiveData(I2Cx);             	// receive byte N-2
		I2C_GenerateSTOP(I2Cx,ENABLE);                  // program stop
		__enable_irq();

		*buf++ = I2C_ReceiveData(I2Cx);             	// receive byte N-1
		// wait for byte N
		Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)); 
		*buf++ = I2C_ReceiveData(I2Cx);
		bufSize = 0;
	}

	// Wait for stop
	Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
	return true;
errReturn:
	I2C_GenerateSTOP(I2Cx, ENABLE);
	LED=1;
  // Any cleanup here
	return false;
}



static bool IMU_I2C_Write(I2C_TypeDef* I2Cx, uint8_t SlaveAddress, uint8_t RegAddress,uint8_t data)
{
	__IO uint32_t Timeout = 0;
//	 Enable Error IT (used in all modes: DMA, Polling and Interrupts
	//    I2Cx->CR2 |= I2C_IT_ERR;
	Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	// Intiate Start Sequence
	I2C_GenerateSTART(I2Cx, ENABLE);
	Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send Address  EV5
	I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Transmitter);
	Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	// EV6
	// Write first byte EV8_1
	//		I2C_SendData(I2Cx,SlaveAddress);
	//		Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));  
	I2C_SendData(I2Cx,RegAddress);
	Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));  
	I2C_SendData(I2Cx,data);
	Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));  
	I2C_GenerateSTOP(I2Cx, ENABLE);
	Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
    return true;
errReturn:
	I2C_GenerateSTOP(I2Cx, ENABLE);
	LED=1;
    return false;
}

*/

/*
static void I2C_LowLevel_Init(I2C_TypeDef* I2Cx)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	// Enable GPIOB clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Configure I2C clock and GPIO
	GPIO_StructInit(&GPIO_InitStructure);
	if (I2Cx == I2C1)
	{
//		 I2C1 clock enable 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//		 I2C1 SDA and SCL configuration 

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

//		 I2C1 Reset 

		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	}
	else
	{
//         I2C2 clock enable 
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//         I2C1 SDA and SCL configuration 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

//       I2C2  Reset 
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
      }

//     Configure I2Cx 

    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDR;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_Init(I2C1,&I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);
}
*/

void reset_calibrate_mag(void)
{
	MxMax=MyMax=MzMax=(int16_t)-32760;
	MxMin=MyMin=MzMin=(int16_t)32760;
}

void Calibrate_Mag(void)
{
	int16_t temp[3];
	int16_t mag[3];
	int16_t mag16_buf[3];
	LED=1;
	if(!READ_MPU9250_RawMag(mag16_buf))return;
	mag[0]=(int16_t)(mag16_buf[0]*Mag_ASA[0])*MxScale;
	mag[1]=(int16_t)(mag16_buf[1]*Mag_ASA[1])*MyScale;
	mag[2]=(int16_t)(mag16_buf[2]*Mag_ASA[2])*MzScale;
	if(MxMin > mag16_buf[0])MxMin=mag16_buf[0];
	if(MyMin > mag16_buf[1])MyMin=mag16_buf[1];
	if(MzMin > mag16_buf[2])MzMin=mag16_buf[2];
	
	if(MxMax < mag16_buf[0])MxMax=mag16_buf[0];
	if(MyMax < mag16_buf[1])MyMax=mag16_buf[1];
	if(MzMax < mag16_buf[2])MzMax=mag16_buf[2];
	UART1_PublishMotion(temp,temp,mag);

}

void Calibrate_SaveMag(void)
{
	MxOffset=IMUconfig.Mx_offset=(MxMax+MxMin)/2.0f*Mag_ASA[0];
	MyOffset=IMUconfig.My_offset=(MyMax+MyMin)/2.0f*Mag_ASA[1];
	MzOffset=IMUconfig.Mz_offset=(MzMax+MzMin)/2.0f*Mag_ASA[2];
	IMUconfig.Mx_scale=MxScale;//=1
	IMUconfig.My_scale=MyScale;//=(MyMax-MyMin)/(MxMax-MxMin)
	IMUconfig.Mz_scale=MzScale;//=(MzMax+MzMin)/(MxMax-MxMin)
	Write_IMUconfig(&IMUconfig);
	MxMax=MyMax=MzMax=-32760;
	MxMin=MyMin=MzMin=32760;
	LED=0;
}

void reset_calibrate_Acc(void)
{

}

void reset_calibrate_Gyro(void)
{
	
}

void Calibrate_Acc(void)
{

}

void Calibrate_Gyro(void)
{
	int16_t acc16_buf[3],gyro16_buf[3];
	int16_t i=0;
	int32_t sum[3];
	while(i<100)
	{
		if(READ_MPU9250_RawAccGyro(acc16_buf,gyro16_buf))
		{
			sum[0]+=gyro16_buf[0];
			sum[1]+=gyro16_buf[1];
			sum[2]+=gyro16_buf[2];
			i++;
		}	
	}
	IMUconfig.Gx_offset=GxOffset = sum[0]/i;
	IMUconfig.Gy_offset=GyOffset = sum[1]/i;
	IMUconfig.Gz_offset=GzOffset = sum[2]/i;
}

void Calibrate_SaveAcc(void)
{

}

void Calibrate_SaveGyro(void)
{
	Write_IMUconfig(&IMUconfig);
}


void MPU9250_Init(void)
{
	uint8_t ASA_RawData[3];
	int8_t i;
	float Acc_Scale,GYRO_Scale,dt;
	float acc_temp[3],gyro_temp[3],mag_temp[3];
	GPIO_InitTypeDef  GPIO_InitStructure; 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin =12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,12);
//	I2C_LowLevel_Init(I2C1);
	IIC_Init(); 
	//reset all gyro,accel,temp digital signal path
	IMU_I2C_Write(IMU_ACC_ADDRESS,USER_CTRL,IMU_SIG_COND_RST);
	//reset the internal registers and restores the default settings
	IMU_I2C_Write(IMU_GYR_ADDRESS,PWR_MGMT_1,IMU_H_Reset);
	delay_ms(100);
//LED=0;
	//auto select the best available clock source
	IMU_I2C_Write(IMU_GYR_ADDRESS,PWR_MGMT_1,IMU_CLKSEL_AUTO);
	//	IMU_I2C_Write(I2C1,IMU_GYR_ADDRESS,PWR_MGMT_1,IMU_CLKSEL_INTERNAL);
	//set interrupt status is cleared if any read operation is performed
	IMU_I2C_Write(IMU_ACC_ADDRESS,INT_PIN_CFG,INT_ANY_RD_CL);
	//enable raw sensor data ready interrupt to propagete to interrupt pin
	IMU_I2C_Write(IMU_ACC_ADDRESS,INT_ENABLE,INT_RAW_RDY_EN);
//LED=0;	
	// enable the FSYNC function,set gyroscope bandwidth 
	// and set temperature bandwidth 
	IMU_I2C_Write(IMU_ACC_ADDRESS,CONFIG,DLPF_CFG_98hz);
	//set sample rate 1Khz: sample_rate=internal_sample_rate/(1+SMPLRT_DIV)
	IMU_I2C_Write(IMU_GYR_ADDRESS,SMPLRT_DIV, 0x07);
	//set gyro full scale 1000dps chose internal sample 1K
	IMU_I2C_Write(IMU_GYR_ADDRESS,GYRO_CONFIG, GYRO_SCALE_1000 | FCHOICE_11);
	GYRO_Scale =(1000.0f/32768.0f);
	
	//set accel full scale +-8G
	IMU_I2C_Write(IMU_ACC_ADDRESS,ACCEL_CONFIG,ACCEL_SCALE_8G);
	Acc_Scale=8000.0/32768.0;
	//set DLPF , ACCEL_FCHOICE=1,accel bandwidth 218HZ ,
	IMU_I2C_Write(IMU_ACC_ADDRESS,ACCEL_CONFIG_2, ACCEL_FCHOICE|A_DLPF_CFG_98hz);
	//SET MST
	IMU_I2C_Write(IMU_ACC_ADDRESS,USER_CTRL,IMU_I2C_MSTDISEN);
	//interrupt pin level held until interrupt status is cleared
	IMU_I2C_Write(IMU_ACC_ADDRESS,INT_PIN_CFG,BYPASS_EN|INT_LATCH_EN);
	
	//Initial AK8963
	IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL2,MAG_SRST);
	delay_ms(20);
IMU_I2C_Read(IMU_MAG_ADDRESS,MAG_WIA,ASA_RawData,1);
printf("WIA=%x \r\n",ASA_RawData[0]);
	IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL1,MODE_POWER_DOWN);
	delay_ms(20);
	IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL1,MODE_FUSE_ROM |MAG_BIT_16);
	delay_ms(20);
	IMU_I2C_Read(IMU_MAG_ADDRESS,MAG_ASAX,ASA_RawData,3);
	

printf("ASA=%x : %x : %x \r\n",ASA_RawData[0],ASA_RawData[1],ASA_RawData[2]);
	Mag_ASA[0] = (float)((ASA_RawData[0]-128)/256.0f+1);
	Mag_ASA[1] = (float)((ASA_RawData[1]-128)/256.0f+1);
	Mag_ASA[2] = (float)((ASA_RawData[2]-128)/256.0f+1);
printf("ASA=%f : %f : %f \r\n",Mag_ASA[0],Mag_ASA[1],Mag_ASA[2]);
	IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL1,MODE_POWER_DOWN);
	delay_ms(20);
	IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL1,MODE_MEASURE_SINGLE | MAG_BIT_16);//at least delay 100us
//	IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL1,MODE_CONTINUOUS_M2 | MAG_BIT_16);
	delay_ms(20);
	Load_IMUconfig(&IMUconfig);
	
	AxOffset=IMUconfig.Ax_offset;
	AyOffset=IMUconfig.Ay_offset;
	AzOffset=IMUconfig.Az_offset;
	GxOffset=IMUconfig.Gx_offset;
	GyOffset=IMUconfig.Gy_offset;
	GzOffset=IMUconfig.Gz_offset;
	MxOffset=IMUconfig.Mx_offset;
	MyOffset=IMUconfig.My_offset;
	MzOffset=IMUconfig.Mz_offset;
	AxScale =IMUconfig.Ax_scale*Acc_Scale;
	AyScale =IMUconfig.Ay_scale*Acc_Scale;
	AzScale =IMUconfig.Az_scale*Acc_Scale;
	GxScale =IMUconfig.Gx_scale*GYRO_Scale;
	GyScale =IMUconfig.Gy_scale*GYRO_Scale;
	GzScale =IMUconfig.Gz_scale*GYRO_Scale;
	MxScale =IMUconfig.Mx_scale;
	MyScale =IMUconfig.My_scale;
	MzScale =IMUconfig.Mz_scale;
	MxMax=MyMax=MzMax=-32760;
	MxMin=MyMin=MzMin=32760;
	Initial_Timer2AndTimer3();
//LED=0;
printf("MxyzOffset=%f: %f: %f \r\n",MxOffset,MyOffset,MzOffset);
printf("MxyzScale=%f: %f: %f \r\n",MxScale,MyScale,MzScale);
	for(i=0;i<50;i++)get_MPU9250_DATA(gyro_temp,acc_temp,mag_temp,&dt);
}
	

void READ_MPU9250_TEMP(uint8_t * temp_buf)
{
	IMU_I2C_Read(IMU_ACC_ADDRESS,TEMP_OUT_H,temp_buf,2);
}


static void MeanFilter_Mpu6050(int16_t* acc16_buf,int16_t* gyro16_buf)
{
	int8_t i,j;
	int sum;
	if(MPU6050_FIFO_PTR>9)MPU6050_FIFO_PTR=0;
	MPU6050_FIFO[0][MPU6050_FIFO_PTR]=acc16_buf[0];
	MPU6050_FIFO[1][MPU6050_FIFO_PTR]=acc16_buf[1];
	MPU6050_FIFO[2][MPU6050_FIFO_PTR]=acc16_buf[2];
	
	MPU6050_FIFO[3][MPU6050_FIFO_PTR]=gyro16_buf[0];
	MPU6050_FIFO[4][MPU6050_FIFO_PTR]=gyro16_buf[1];
	MPU6050_FIFO[5][MPU6050_FIFO_PTR]=gyro16_buf[2];
	MPU6050_FIFO_PTR++;
	for(j=0;j<6;j++)
	{
		sum=0;
		for(i=0;i<10;i++)
		{
			sum+=MPU6050_FIFO[j][i];
		}
		MPU6050_FIFO[j][10]=sum/10;
	}
}

/*
static void MeanFilter_AK8963(int16_t* mag16_buf)
{
	int8_t i,j;
	int sum;
	if(AK8963_FIFO_PTR>3)AK8963_FIFO_PTR=0;
	AK8963_FIFO[0][AK8963_FIFO_PTR]=mag16_buf[0];
	AK8963_FIFO[1][AK8963_FIFO_PTR]=mag16_buf[1];
	AK8963_FIFO[2][AK8963_FIFO_PTR]=mag16_buf[2];
	AK8963_FIFO_PTR++;
	for(j=0;j<3;j++)
	{
		sum=0;
		for(i=0;i<4;i++)
		{
			sum+=AK8963_FIFO[j][i];
		}
		mag16_buf[j]=AK8963_FIFO[j][4]=sum/4;
	}
	
//	AK8963_FIFO[0][4]=mag16_buf[0];
//	AK8963_FIFO[1][4]=mag16_buf[1];
//	AK8963_FIFO[2][4]=mag16_buf[2];	
	
}
*/
bool READ_MPU9250_RawAccGyro(int16_t* acc16_buf,int16_t* gyro16_buf)
{
	uint8_t BUF[14];
	if(IMU_I2C_Read(IMU_ACC_ADDRESS,ACCEL_OUT,BUF,14))
	{
		acc16_buf[0]=(int16_t)(((int16_t)BUF[0]<<8) | BUF[1]);		
		acc16_buf[1]=(int16_t)(((int16_t)BUF[2]<<8) | BUF[3]);	
		acc16_buf[2]=(int16_t)(((int16_t)BUF[4]<<8) | BUF[5]);
		
		gyro16_buf[0]=(int16_t)(((int16_t)BUF[8] <<8) | BUF[9]);
		gyro16_buf[1]=(int16_t)(((int16_t)BUF[10]<<8) | BUF[11]);
		gyro16_buf[2]=(int16_t)(((int16_t)BUF[12]<<8) | BUF[13]);
		MeanFilter_Mpu6050(acc16_buf,gyro16_buf);
		acc16_buf[0]= MPU6050_FIFO[0][10];
		acc16_buf[1]= MPU6050_FIFO[1][10];
		acc16_buf[2]= MPU6050_FIFO[2][10];
		
		gyro16_buf[0]= MPU6050_FIFO[3][10];
		gyro16_buf[1]= MPU6050_FIFO[4][10];
		gyro16_buf[2]= MPU6050_FIFO[5][10];
		return true;
	}
	return false;
}
bool READ_MPU9250_RawMag(int16_t* mag16_buf)
{ 
	uint8_t BUF[8];
	if(!IMU_I2C_Read(IMU_MAG_ADDRESS,MAG_ST1,BUF,8))return false;
	if((BUF[0]&0X01)/*&&(!(BUF[0]&0X02))*/)
	{
//LED=1;
//		if(!IMU_I2C_Read(I2C1,IMU_MAG_ADDRESS,MAG_OUT,&BUF[1],7))return false;
//		if(!(BUF[7]&0x08))
//		{
			mag16_buf[0]=(int16_t)(((int16_t)BUF[2]<<8) | BUF[1]);		
			mag16_buf[1]=(int16_t)(((int16_t)BUF[4]<<8) | BUF[3]);
			mag16_buf[2]=(int16_t)(((int16_t)BUF[6]<<8) | BUF[5]);
//MeanFilter_AK8963(mag16_buf);
			IMU_I2C_Write(IMU_MAG_ADDRESS,MAG_CRTL1,MODE_MEASURE_SINGLE | MAG_BIT_16);//at least delay 100us		
			return true;
//		}
		
	}
	return false;
}

bool get_MPU9250_DATA(float* gyro,float* acc,float* mag,float* dt)
{
	int16_t acc16_buf[3],gyro16_buf[3],mag16_buf[3];
	int32_t now;
	if(!READ_MPU9250_RawAccGyro(acc16_buf,gyro16_buf))return false;
	acc[0]=(float)(MPU6050_FIFO[0][10]-AxOffset) * AxScale;
	acc[1]=(float)(MPU6050_FIFO[1][10]-AyOffset) * AyScale;
	acc[2]=(float)(MPU6050_FIFO[2][10]-AzOffset) * AzScale;
	
	gyro[0]=(float)(MPU6050_FIFO[3][10]-GxOffset) * GxScale;
	gyro[1]=(float)(MPU6050_FIFO[4][10]-GyOffset) * GyScale;
	gyro[2]=(float)(MPU6050_FIFO[5][10]-GzOffset) * GzScale;
	if(!READ_MPU9250_RawMag(mag16_buf))return false;
	mag[0]=(float)(mag16_buf[0]*Mag_ASA[0]-MxOffset)*MxScale;
	mag[1]=(float)(mag16_buf[1]*Mag_ASA[1]-MyOffset)*MyScale;
	mag[2]=(float)(mag16_buf[2]*Mag_ASA[2]-MzOffset)*MzScale;
	
	
	now = micros();
	if(now<lastUpdate)
	{
		*dt =  ((float)(now + (0xffff- lastUpdate)) / 1000000.0f);
	}
	else
	{
		*dt =  ((float)(now - lastUpdate) / 1000000.0f);
	}
	lastUpdate = now;
	return true;
}

