/***************************************************************************
Copyright (c) <2018>, <Shawn Zhang>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the <organization>.
4. Neither the name of the <organization> nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY <Shawn Zhang> ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <Shawn Zhang> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*******************************************************************************/
#include "Usart.h"
#include "Encoder.h"
#include "PWM.h"

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
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
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

void Usart1_Init(u16 bound)
{
	GPIO_InitTypeDef     GPIO_Init_Usart1;
	USART_InitTypeDef    USART_Init_Usart1;
	NVIC_InitTypeDef     NVIC_Init_Usart1;
	
	
	
	
	//�ٴ���ʱ��ʹ��,GPIOʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//���������Ÿ�����ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	//��GPIO��ʼ������
	GPIO_Init_Usart1.GPIO_Mode=GPIO_Mode_AF;//���ù���ģʽ
	GPIO_Init_Usart1.GPIO_OType=GPIO_OType_PP;//���츴�����
	GPIO_Init_Usart1.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_Init_Usart1.GPIO_PuPd=GPIO_PuPd_UP;//����
	GPIO_Init_Usart1.GPIO_Speed=GPIO_Fast_Speed;//50MHz
	GPIO_Init(GPIOA, &GPIO_Init_Usart1);
	
	//�ܴ��ڲ�����ʼ��
	USART_Init_Usart1.USART_BaudRate= bound;//����������
	USART_Init_Usart1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init_Usart1.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���պͷ��ͷ�ʽ
	USART_Init_Usart1.USART_Parity=USART_Parity_No;//��У��λ
	USART_Init_Usart1.USART_StopBits=USART_StopBits_1;//һλֹͣλ
	USART_Init_Usart1.USART_WordLength=USART_WordLength_8b;//���ݳ���Ϊ8λ
	USART_Init(USART1, &USART_Init_Usart1);
	
	
	//��NVIC���ã�ʹ�ܴ��ڽ��������ж�
	NVIC_Init_Usart1.NVIC_IRQChannel=USART1_IRQn;//����1�ж�����
	NVIC_Init_Usart1.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init_Usart1.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�����
	NVIC_Init_Usart1.NVIC_IRQChannelSubPriority=2;//��Ӧ���ȼ�����
	NVIC_Init(&NVIC_Init_Usart1);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//���յ����ݲ����жϣ���RXNE�����ݼĴ����ǿգ�
	
	//��ʹ�ܴ���
	USART_Cmd(USART1, ENABLE);
	
}

void USART1_IRQHandler(void)
{
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)//�жϵ�ȷ���յ����µ�����
	{
		USART1_RX_FLAG=1;
		USART1_RX_BUF = USART_ReceiveData(USART1);
		Motor_PWM_Creater((float)USART1_RX_BUF,Foreward_PWM);
	}
	//USART_SendData(USART1, res);
	
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);//����жϱ�־λ
	
}





