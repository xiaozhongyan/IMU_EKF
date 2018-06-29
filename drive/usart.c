#include "usart.h"


#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0); //循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

	
u8 USART1_RX_BUF[USART1_REC_LEN];     //接收缓冲,最大USART1_REC_LEN个字节.
u8 USART1_RX_DATA[USART1_REC_LEN];

u8 USART1_RX_STA=0;       //串口1接收状态标记	 
u16 USART1_RX_SIZE=0;			//串口1接收计数大小
//u16 RX1_FRAME_MAX=0;			//串口1接收帧大小


  
void uart1_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.串口时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_USART1_GPIOx, ENABLE);	//使能USART1，GPIOA时钟
  //2.串口复位
	//USART_DeInit(USART1);
	
	//3.串口引脚配置
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(USART1_TX_GPIOx, &GPIO_InitStructure);//初始化GPIOA.9   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(USART1_RX_GPIOx, &GPIO_InitStructure);//初始化GPIOA.10  

  //4.Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //5.USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	//6.开启串口响应中断
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	//USART_ITConfig(USART1, USART_IT_TC, ENABLE);//开启串口发送中断
	//7.串口使能
  USART_Cmd(USART1, ENABLE);                    //使能串口1 
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
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
	while((USART1->SR&0X40)==0); //循环发送,直到发送完毕   
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


