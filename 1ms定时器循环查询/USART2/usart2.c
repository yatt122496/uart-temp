#include "usart2.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
									  
////////////////////////////////////////////////////////////////////////////////// 	   

//���ڷ��ͻ����� 	
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
#ifdef USART2_RX_EN   								//���ʹ���˽���   	  
//���ڽ��ջ����� 	
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.

u8 USART2_RX_CHECK=0;



//ͨ���жϽ�������2���ַ�֮���ʱ������10ms�������ǲ���һ������������.
//���2���ַ����ռ������10ms,����Ϊ����1����������.Ҳ���ǳ���10msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 USART2_RX_STA=0;
static volatile u8 USART2_RX_CNT=0;
void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//���յ�����
	{
		res =USART_ReceiveData(USART2);
		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//�����Խ�������
		{
			USART2_RX_CNT=0;
			if(USART2_RX_STA==0)USART2_RX_CHECK=1;
//			TIM_SetCounter(TIM4,0);//���������
//			if(USART2_RX_STA==0)TIM4_Set(1);	 	//ʹ�ܶ�ʱ��4���ж�
			USART2_RX_BUF[USART2_RX_STA++]=res;		//��¼���յ���ֵ
		}else
		{
			USART2_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
		}
	}
}
//��ʱ��4�жϷ������		    
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//�Ǹ����ж�
	{
		if(USART2_RX_CHECK)
		{
			USART2_RX_CNT++;
			if(USART2_RX_CNT==10)
			{
				USART2_RX_STA|=1<<15;	//��ǽ������
				USART2_RX_CNT=0;
				USART2_RX_CHECK=0;	
			}
		}
//		USART2_RX_STA|=1<<15;	//��ǽ������
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx�����жϱ�־    
//		TIM4_Set(0);			//�ر�TIM4  
	}
}
//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void USART2_Init(u32 bound)
{  

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

 	USART_DeInit(USART2);  //��λ����2
		 //USART2_TX   PA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA2
   
    //USART2_RX	  PA.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA3
	
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  
	USART_Init(USART2, &USART_InitStructure); //��ʼ������	2
  
	//����������
 //	USART2->BRR=(pclk1*1000000)/(bound);// ����������	 
	//USART2->CR1|=0X200C;  	//1λֹͣ,��У��λ.
//	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  	//ʹ�ܴ���2��DMA����
//	UART_DMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)USART2_TX_BUF);//DMA1ͨ��7,����Ϊ����2,�洢��ΪUSART2_TX_BUF 
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���
	

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART2_RX_STA=0;		//����
	
	TIM4_Init(9,7199);		//10ms�ж�
//	TIM4_Set(0);			//�رն�ʱ��4

}
////����2,printf ����
////ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
//void u2_printf(char* fmt,...)
//{
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf((char*)USART2_TX_BUF,fmt,ap);
//	va_end(ap);
//	while(DMA_GetCurrDataCounter(DMA1_Channel7)!=0);	//�ȴ�ͨ��7�������
//	UART_DMA_Enable(DMA1_Channel7,strlen((const char*)USART2_TX_BUF)); 	//ͨ��dma���ͳ�ȥ
//}
//����2,printf ����
//ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
void u2_printf(char* fmt,...)
{
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART2_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	  USART_SendData(USART2,USART2_TX_BUF[j]); 
	} 
}


//ͨ�ö�ʱ���жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��		 
void TIM4_Init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM4��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx
	
}
#endif		 



