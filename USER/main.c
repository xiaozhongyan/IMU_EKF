#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"

#include "IMU_EKF.h"
#include "mpu9250.h"
#include "Quaternion.h"

#define Gyro_init  		0xE0
#define High_init  		0xE2
#define Mag_Calib		0xE3
#define Mag_Calib_save  0xE1

void sys_Initial()
{
	float Mag_Inclination;
	SystemInit();
	delay_us(10000);
	LED_Init();
//LED=0;
	uart1_init(B115200);
//LED=0;
	MPU9250_Init();
LED=0;
	Init_ekf(Mag_Inclination);
}

int main()
{
	int i;
	float ACC_DATA[3],GYRO_DATA[3],MAG_DATA[3],dt;
	float V[3];
	float rpy[3];
	unsigned char Cammand=0;
	int16_t acc_temp[3],gyro_temp[3],mag_temp[3];
	sys_Initial();
	while(1)
	{	
/*
//		LED=0;
		if(USART1_RX_STA & USART1_GET_OVER)Cammand=get_command();
		if(Cammand)
		{
//			LED=0;
			switch(Cammand)
			{
				case Mag_Calib: 
					Calibrate_Mag();
					break;			
				case Mag_Calib_save: 
					Calibrate_SaveMag();
					Cammand=0;
					break;
				default:
					Cammand=0;
					LED=0;
			}
			LED=0;
			continue;
		}
*/

//LED=0;
		if(get_MPU9250_DATA(GYRO_DATA,ACC_DATA,MAG_DATA,&dt))
		{
//			EKFupdata(ACC_DATA,GYRO_DATA,MAG_DATA,V,dt);
//			QUAT_GetAngle(rpy);
//LED=0;
//			delay_ms(10);
			for(i=0;i<3;i++)
			{
				acc_temp[i]=(int16_t)(ACC_DATA[i]);
				gyro_temp[i]=(int16_t)(GYRO_DATA[i]);
				mag_temp[i]=(int16_t)(MAG_DATA[i]);
			}
			
//			printf("mag=%f : %f : %f \r\n",MAG_DATA[0],MAG_DATA[1],MAG_DATA[2]);
//			printf("mag=%d : %d : %d \r\n",mag_temp[0],mag_temp[1],mag_temp[2]);
//			printf(": DT=%f",dt);
//			mag_temp[0]=0;
			UART1_PublishMotion(acc_temp,gyro_temp,mag_temp);
		}
//delay_ms(4);
//LED=1;
//delay_ms(20);
	}
}

