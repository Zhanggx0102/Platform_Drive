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
#include "LED.h"
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
void LED_Init()
{
	GPIO_InitTypeDef GPIO_Init_LED; 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//开启GPIOF时钟
	
	GPIO_Init_LED.GPIO_Mode=GPIO_Mode_OUT;//普通输出模式
	GPIO_Init_LED.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_Init_LED.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;//引脚
	GPIO_Init_LED.GPIO_PuPd=GPIO_PuPd_UP;//上拉
	GPIO_Init_LED.GPIO_Speed=GPIO_Speed_100MHz;	
	GPIO_Init(GPIOF, &GPIO_Init_LED);
	
	GPIO_SetBits(GPIOF, GPIO_Pin_9|GPIO_Pin_10);//拉高引脚，灯灭
	
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
void LED_OFF(void)
{
	GPIO_SetBits(GPIOF, GPIO_Pin_9|GPIO_Pin_10);//拉高引脚，灯灭
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
void LED_ON(void)
{
	GPIO_ResetBits(GPIOF, GPIO_Pin_9|GPIO_Pin_10);//拉高引脚，灯灭
}


