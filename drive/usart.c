#include "usart.h"


#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0); //ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

	
u8 USART1_RX_BUF[USART1_REC_LEN];     //���ջ���,���USART1_REC_LEN���ֽ�.
u8 USART1_RX_DATA[USART1_REC_LEN];

u8 USART1_RX_STA=0;       //����1����״̬���	 
u16 USART1_RX_SIZE=0;			//����1���ռ�����С
//u16 RX1_FRAME_MAX=0;			//����1����֡��С


  
void uart1_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_USART1_GPIOx, ENABLE);	//ʹ��USART1��GPIOAʱ��
  //2.���ڸ�λ
	//USART_DeInit(USART1);
	
	//3.������������
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(USART1_TX_GPIOx, &GPIO_InitStructure);//��ʼ��GPIOA.9   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(USART1_RX_GPIOx, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //4.Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //5.USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	//6.����������Ӧ�ж�
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	//USART_ITConfig(USART1, USART_IT_TC, ENABLE);//�������ڷ����ж�
	//7.����ʹ��
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		data=USART_ReceiveData(USART1);
		if(data==0xa5)
		{ 
			USART1_RX_STA|=USART1_GET_HEAD;
			USART1_RX_BUF[USART1_RX_SIZE++]=data;
		}
		else if(data==0x5a)
		{ 
			if(USART1_RX_STA & USART1_GET_HEAD)
			{ 
				USART1_RX_SIZE=0;				
				USART1_RX_STA&=~USART1_GET_OVER;
			}
			else
				USART1_RX_BUF[USART1_RX_SIZE++]=data;
			USART1_RX_STA&=~USART1_GET_HEAD;
		}
		else
		{ 
			USART1_RX_BUF[USART1_RX_SIZE++]=data;
			USART1_RX_STA&=~USART1_GET_HEAD;
			if(USART1_RX_SIZE==USART1_RX_BUF[0])
			{
				USART1_RX_STA|=USART1_GET_OVER;
			}
		}
		if(USART1_RX_SIZE>40)
			USART1_RX_SIZE=0;
		/* Clear the USART1 transmit interrupt */
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

char get_command()
{
	if(USART1_RX_STA & USART1_GET_OVER)
	{
		USART1_RX_STA =0;
		return USART1_RX_DATA[1];
	}
	return 0;
}

void UART1_Put_Char(unsigned char DataToSend)
{
	while((USART1->SR&0X40)==0); //ѭ������,ֱ���������   
    USART1->DR = DataToSend;
}
void UART1_PublishMotion(int16_t* acc,int16_t* gyro,int16_t* mag)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+8);
	UART1_Put_Char(0xA2);

	if(acc[0]<0)acc[0]=32768-acc[0];
	ctemp=acc[0]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=acc[0];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(acc[1]<0)acc[1]=32768-acc[1];
	ctemp=acc[1]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=acc[1];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(acc[2]<0)acc[2]=32768-acc[2];
	ctemp=acc[2]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=acc[2];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gyro[0]<0)gyro[0]=32768-gyro[0];
	ctemp=gyro[0]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gyro[0];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gyro[1]<0)gyro[1]=32768-gyro[1];
	ctemp=gyro[1]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gyro[1];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gyro[2]<0)gyro[2]=32768-gyro[2];
	ctemp=gyro[2]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gyro[2];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(mag[0]<0)mag[0]=32768-mag[0];
	ctemp=mag[0]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=mag[0];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(mag[1]<0)mag[1]=32768-mag[1];
	ctemp=mag[1]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=mag[1];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(mag[2]<0)mag[2]=32768-mag[2];
	ctemp=mag[2]>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=mag[2];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}


