#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h"

#define EN_USART1 			1		//使能（1）/禁止（0）串口1接收


//#define GET_FRAME				0x8000 //接收一帧数据
#define USART1_GET_HEAD  			0x80
#define USART1_GET_OVER    			0x40
#define B9600				9600
#define B14400 			14400
#define B19200			19200
#define B38400			38400
#define B43000			43000
#define B57600			57600
#define B76800			76800
#define B115200			115200
#define B128000			128000

//使能串口1模块
//定义串口接收中断缓存大小
#define USART1_REC_LEN  			40  	//定义最大接收缓冲字节数 20
//定义发送引脚
#define USART1_TX_GPIOx				GPIOA
#define USART1_TX_PIN				GPIO_Pin_9
//定义接收引脚
#define USART1_RX_GPIOx				GPIOA
#define USART1_RX_PIN				GPIO_Pin_10
//定义收发引脚时钟总线
#define RCC_USART1_GPIOx 			RCC_APB2Periph_GPIOA

extern u8 USART1_RX_STA;         		//接收状态标记	
//extern u8* USART1_RX_DATA;						//外部缓冲复本指针
//extern u16 RX_SIZE;										//接收数据帧大小
//如果想串口中断接收，请不要注释以下宏定义
void uart1_init(u32 bound);
//void uart1_set_rxbuf(u8* rx_buf);
void uart1_set_rxbuf(u8* rx_buf,u16 rx_bufSize_max);
void UART1_PublishMotion(int16_t* acc,int16_t* gyro,int16_t* mag);
char get_command(void);

// 使能串口2模块


#endif
