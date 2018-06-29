#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h"

#define EN_USART1 			1		//ʹ�ܣ�1��/��ֹ��0������1����


//#define GET_FRAME				0x8000 //����һ֡����
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

//ʹ�ܴ���1ģ��
//���崮�ڽ����жϻ����С
#define USART1_REC_LEN  			40  	//���������ջ����ֽ��� 20
//���巢������
#define USART1_TX_GPIOx				GPIOA
#define USART1_TX_PIN				GPIO_Pin_9
//�����������
#define USART1_RX_GPIOx				GPIOA
#define USART1_RX_PIN				GPIO_Pin_10
//�����շ�����ʱ������
#define RCC_USART1_GPIOx 			RCC_APB2Periph_GPIOA

extern u8 USART1_RX_STA;         		//����״̬���	
//extern u8* USART1_RX_DATA;						//�ⲿ���帴��ָ��
//extern u16 RX_SIZE;										//��������֡��С
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 bound);
//void uart1_set_rxbuf(u8* rx_buf);
void uart1_set_rxbuf(u8* rx_buf,u16 rx_bufSize_max);
void UART1_PublishMotion(int16_t* acc,int16_t* gyro,int16_t* mag);
char get_command(void);

// ʹ�ܴ���2ģ��


#endif
