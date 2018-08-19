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
/* Description ------------------------------------------------------------------*/
/*******************************************************************************
* Author:          --Shawn
* Time:						 --2016.9.6
* Last Modify:     --2018
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Delay.h"
#include "LED.h"
#include "Usart.h"
#include "PWM.h"
#include "Encoder.h"
#include "SysTick.h"
#include "PID_Controller.h"

/* Variable ----------------------------------------------------------------- */
volatile u8 USART1_RX_FLAG;//串口接收数据标志
volatile u16 USART1_RX_BUF; //串口接收数据保存

extern float Right_Angular_Velocity;
extern float Right_RPM;
extern int16_t Right_Direction;
static u16 Led_Flag;

extern float a , b;

/* Main Function ----------------------------------------------------------------- */
int main(void)
{
	
/* Init Functions ---------------------------------------------------------------- */
	USART1_RX_FLAG = 0;
	USART1_RX_BUF = 0;
	
/* Init Functions ---------------------------------------------------------------- */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Set NVIC Priority Group
	
	Delay_Init();
	LED_Init();
	Usart1_Init(9600);
	TIM10_PWM_Init(2100-1,4-1);
	Motor_Control_Init();
	Encoders_Init();
	SysTick_Init();

	while(1)
	{
		Led_Flag++;
		if(Led_Flag>=5)
		{
			LED_ON();
			Led_Flag = 0;
		}
		else
		{
			LED_OFF();
		}
		
		Delay_ms(200);
		if(USART1_RX_FLAG ==1)
		{
		printf("Speed: %d %f %f %d %f %f \n",USART1_RX_BUF, a,b,Right_Direction,Right_RPM,Right_Angular_Velocity);
		}
		
	}
}


