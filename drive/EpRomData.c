#include "EpRomData.h"

//struct data_map Config;	//配置信息
//struct IMU_config_data IMUConfig;

void Load_IMUconfig(IMU_config_info* IMUConfig){
	int16_t i;
	int16_t *ptr = &IMUConfig->is_valid;
	int16_t *temp_addr = (int16_t *)PAGE_Config;
	__disable_irq();
	FLASH_Unlock();
	for(i=0 ; i< sizeof(IMU_config_info)/2;i++)
	{
		*ptr = *temp_addr;
		temp_addr++;
		ptr++;
	}
	FLASH_Lock();
	__enable_irq();
	if(IMUConfig->is_valid!= (int16_t)0x9AA5){ //数据无效 ，此时需要装载默认值。
		IMUConfig->is_valid = 0x9AA5;
		IMUConfig->Gx_offset = 0;
		IMUConfig->Gy_offset = 0;
		IMUConfig->Gz_offset = 0;
		
		IMUConfig->Ax_offset =0;
		IMUConfig->Ay_offset =0;
		IMUConfig->Az_offset =0;
	
		IMUConfig->Mx_offset = 0.0f;
		IMUConfig->My_offset = 0.0f;
		IMUConfig->Mz_offset = 0.0f;
		
		IMUConfig->Gx_scale = 1.0f;
		IMUConfig->Gy_scale = 1.0f;
		IMUConfig->Gz_scale = 1.0f;
		
		IMUConfig->Ax_scale = 1.0f;
		IMUConfig->Ay_scale = 1.0f;
		IMUConfig->Az_scale = 1.0f;		
	
		IMUConfig->Mx_scale =1.0f;
		IMUConfig->My_scale =1.0f;
		IMUConfig->Mz_scale =1.0f;
		
		Write_IMUconfig(IMUConfig);	
	}
}

//将当前配置写入flash
void Write_IMUconfig(IMU_config_info* IMUConfig)
{
	int16_t i;
	int16_t *ptr = &IMUConfig->is_valid;
	uint32_t ptemp_addr = PAGE_Config;
	__disable_irq();
	FLASH_Unlock();
 	FLASH_ErasePage(PAGE_Config); //擦 页
	for(i=0;i<sizeof(IMU_config_info)/2;i++)
	{
	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
	 	ptemp_addr+=2;
	}
	FLASH_Lock();
	__enable_irq();
}

