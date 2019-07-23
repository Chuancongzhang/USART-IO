#include "stm32f10x.h"
#include "delay.h"

/**
* ������ڵ�ʵ�֣�GPIOģ�⴮���շ����ݣ�
* ������:9600    1-8-N
* TXD : PA9
* RXD : PA10
* ʹ���ⲿ�ж϶�RXD���½��ؽ��д�����ʹ�ö�ʱ��4����9600�����ʽ��ж�ʱ���ݽ���
* ����: ����11�����ݣ�Ȼ��ѽ��յ������ݷ��ͳ�ȥ
*/

#define BuadRate_9600	100

u8 len = 0;	// �����ַ�����
u8 USART_buf[11];  //�����ַ����黺����
u8 Tx,Rx;

enum{
	COM_START_BIT,
	COM_D0_BIT,
	COM_D1_BIT,
	COM_D2_BIT,
	COM_D3_BIT,
	COM_D4_BIT,
	COM_D5_BIT,
	COM_D6_BIT,
	COM_D7_BIT,
	COM_STOP_BIT,
};

u8 recvStat = COM_STOP_BIT;
u8 recvData = 0;

void IO_TXD(u8 Data)
{
	u8 i = 0;
	GPIO_ResetBits(GPIOA,GPIO_Pin_9);
	delay_us(BuadRate_9600);
	for(i = 0; i < 8; i++)
	{
		if(Data&0x01)
			GPIO_SetBits(GPIOA,GPIO_Pin_9); 
		else	
			GPIO_ResetBits(GPIOA,GPIO_Pin_9);
		delay_us(BuadRate_9600);
		Data = Data>>1;
	}
	GPIO_SetBits(GPIOA,GPIO_Pin_9); 
	delay_us(BuadRate_9600);
}
	
void USART_Send(u8 *buf, u8 len)
{
	u8 t;
	for(t = 0; t < len; t++)
	{
		IO_TXD(buf[t]);
	}
}
	
 void IOConfig(void)
 {
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 	EXTI_InitTypeDef EXTI_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA 
	 
	 //SoftWare Serial TXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�����������ģʽ�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //��������������Ƶ��50MHz 
  GPIO_Init(GPIOA, &GPIO_InitStructure);	  				
  GPIO_SetBits(GPIOA,GPIO_Pin_9); 						
	 
	 
	//SoftWare Serial RXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);
	EXTI_InitStruct.EXTI_Line = EXTI_Line10;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling; //�½����ж�
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStruct);


	NVIC_InitStructure.NVIC_IRQChannel= EXTI15_10_IRQn ; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
}
 
void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʹ��ʱ��
	
	//��ʱ��TIM4��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�������Զ�װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ��ģʽ
	TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�жϣ���������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  ////��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���		 
}
 
 
 int main(void)
 {		
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	 delay_init();
	 IOConfig();
   TIM4_Int_Init(107, 71);	 //�벨���ʽ��Ƶļ���Ƶ��
	 
  while(1)
	{
		if(len > 10)
		{
			len = 0;
			USART_Send(USART_buf,11);
		}
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line10) != RESET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10) == 0) 	
		{
			if(recvStat == COM_STOP_BIT)
			{
				recvStat = COM_START_BIT;
				TIM_Cmd(TIM4, ENABLE);
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

void TIM4_IRQHandler(void)
{  
	if(TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);	
		 recvStat++;
		if(recvStat == COM_STOP_BIT)
		{
			TIM_Cmd(TIM4, DISABLE);
			USART_buf[len++] = recvData;	
			return;
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10))
		{
			recvData |= (1 << (recvStat - 1));
		}else{
			recvData &= ~(1 << (recvStat - 1));
		}	
  }		
}
