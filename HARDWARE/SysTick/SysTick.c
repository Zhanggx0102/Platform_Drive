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
/* Includes ------------------------------------------------------------------*/
#include "SysTick.h"
#include "Encoder.h"
#include "PID_Controller.h"
#include "Usart.h"

/* Variable ----------------------------------------------------------------- */
volatile u16 SysTick_Interval= 10;//set systick interrupt every SysTick_Interval ms
//volatile u16 Led_Flag;

extern float Right_Angular_Velocity;
extern float Right_RPM;
extern int16_t Right_Direction;
extern volatile PID_Increment_Struct Right_Motor_PID;

extern volatile u16 USART1_RX_BUF; 

extern float a , b;

//float Velocity_Now;

/* Functions ---------------------------------------------------------------- */
/*******************************************************************************
* Function Name  : --SysTick_Init
* Description    : --Init SysTick clock 
									 --Set the interval time as 10ms
									
* Input          : --NULL
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/
void SysTick_Init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//set Systick clck as 8 division HCLK clock£¨168MHz£©21MHz
	SysTick->LOAD = 1000*SysTick_Interval*168/8;  //set reload value as SysTick_Interval ms
	SysTick->VAL = 0x00; //Clear counter
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;  //Enable SYSTICK interupt
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 	//Enable SYSTICK 
}
/*******************************************************************************
* Function Name  : --
* Description    : --
									 --
									 --
									 --
* Input          : --NULL
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/
void SysTick_Handler()
{	
	//read the encoder
	Encoders_Read();
	//Velocity_Now = Right_RPM;
	PID_Controller((float)USART1_RX_BUF,Right_RPM);
	//a = (float)USART1_RX_BUF;
	//b = Right_RPM;
	//a++;
}

