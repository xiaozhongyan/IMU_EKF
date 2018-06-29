#ifndef _EPROMDATA_H_
#define _EPROMDATA_H_

#include "stm32f10x.h"
#define  PAGE_Config    (0x08000000 + 62 * 1024) //将配置信息存放在第62页Flash

struct IMU_config_data{
	int16_t is_valid;
	
	int16_t Gx_offset;
	int16_t Gy_offset;
	int16_t Gz_offset;
	
	int16_t Ax_offset;
	int16_t Ay_offset;
	int16_t Az_offset;	
	
	float Mx_offset;
	float My_offset;
	float Mz_offset;
	
	float  Gx_scale;
	float  Gy_scale;
	float  Gz_scale;
	
	float  Ax_scale;
	float  Ay_scale;
	float  Az_scale;
	
	float  Mx_scale;
	float  My_scale;
	float  Mz_scale;
};
typedef struct IMU_config_data IMU_config_info;
//extern struct IMU_config_data IMUConfig;

void Load_IMUconfig(IMU_config_info* IMUConfig);
void Write_IMUconfig(IMU_config_info* IMUConfig);


#endif
