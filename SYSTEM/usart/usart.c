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
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
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
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

void Usart1_Init(u16 bound)
{
	GPIO_InitTypeDef     GPIO_Init_Usart1;
	USART_InitTypeDef    USART_Init_Usart1;
	NVIC_InitTypeDef     NVIC_Init_Usart1;
	
	
	
	
	//①串口时钟使能,GPIO时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//②设置引脚复用器映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	//③GPIO初始化设置
	GPIO_Init_Usart1.GPIO_Mode=GPIO_Mode_AF;//复用功能模式
	GPIO_Init_Usart1.GPIO_OType=GPIO_OType_PP;//推挽复用输出
	GPIO_Init_Usart1.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_Init_Usart1.GPIO_PuPd=GPIO_PuPd_UP;//上拉
	GPIO_Init_Usart1.GPIO_Speed=GPIO_Fast_Speed;//50MHz
	GPIO_Init(GPIOA, &GPIO_Init_Usart1);
	
	//④串口参数初始化
	USART_Init_Usart1.USART_BaudRate= bound;//波特率设置
	USART_Init_Usart1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init_Usart1.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//接收和发送方式
	USART_Init_Usart1.USART_Parity=USART_Parity_No;//无校验位
	USART_Init_Usart1.USART_StopBits=USART_StopBits_1;//一位停止位
	USART_Init_Usart1.USART_WordLength=USART_WordLength_8b;//数据长度为8位
	USART_Init(USART1, &USART_Init_Usart1);
	
	
	//⑤NVIC设置，使能串口接收数据中断
	NVIC_Init_Usart1.NVIC_IRQChannel=USART1_IRQn;//串口1中断设置
	NVIC_Init_Usart1.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init_Usart1.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级设置
	NVIC_Init_Usart1.NVIC_IRQChannelSubPriority=2;//响应优先级设置
	NVIC_Init(&NVIC_Init_Usart1);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//接收到数据产生中断，（RXNE读数据寄存器非空）
	
	//⑥使能串口
	USART_Cmd(USART1, ENABLE);
	
}

void USART1_IRQHandler(void)
{
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)//判断的确接收到了新的数据
	{
		USART1_RX_FLAG=1;
		USART1_RX_BUF = USART_ReceiveData(USART1);
		Motor_PWM_Creater((float)USART1_RX_BUF,Foreward_PWM);
	}
	//USART_SendData(USART1, res);
	
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志位
	
}





