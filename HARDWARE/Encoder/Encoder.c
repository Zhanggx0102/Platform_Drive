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
#include "Encoder.h"
#include "SysTick.h"
/* Definition ----------------------------------------------------------------- */
#define Pulse_perCiecle  1440
#define PI  3.141593

//definitions for the quadrature encoder pins
// Right Motor Channels
#define Right_EncoderA_PIN               GPIO_Pin_6
#define Right_EncoderA_GPIO_PORT         GPIOB
#define Right_EncoderA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define Right_EncoderA_SOURCE            GPIO_PinSource6
#define Right_EncoderA_AF                GPIO_AF_TIM4

#define Right_EncoderB_PIN               GPIO_Pin_7
#define Right_EncoderB_GPIO_PORT         GPIOB
#define Right_EncoderB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define Right_EncoderB_SOURCE            GPIO_PinSource7
#define Right_EncoderB_AF                GPIO_AF_TIM4

// determine the timers to use
#define Right_Encoder_TIMER              TIM4
#define Right_Encoder_TIMER_CLK          RCC_APB1Periph_TIM4
#define Right_Encoder_TIMWE_IRQn				 TIM4_IRQn
#define Right_Encoder_Count()            ENCR_TIMER->CNT

// Left Motor Channels
#define Left_EncoderA_PIN               GPIO_Pin_6
#define Left_EncoderA_GPIO_PORT         GPIOC
#define Left_EncoderA_GPIO_CLK          RCC_AHB1Periph_GPIOC
#define Left_EncoderA_SOURCE            GPIO_PinSource6
#define Left_EncoderA_AF                GPIO_AF_TIM3

#define Left_EncoderB_PIN               GPIO_Pin_7
#define Left_EncoderB_GPIO_PORT         GPIOC
#define Left_EncoderB_GPIO_CLK          RCC_AHB1Periph_GPIOC
#define Left_EncoderB_SOURCE            GPIO_PinSource7
#define Left_EncoderB_AF                GPIO_AF_TIM3

// determine the timers to use
#define Left_Encoder_TIMER              TIM3
#define Left_Encoder_TIMER_CLK          RCC_APB1Periph_TIM3
#define Left_Encoder_TIMWE_IRQn				  TIM3_IRQn
#define Left_Encoder_Count()            ENCR_TIMER->CNT

/* Variable ----------------------------------------------------------------- */

//volatile u32 a , b;//debug

extern u16 SysTick_Interval;

volatile uint8_t Right_Encoder_Interrupt_flag;
volatile uint8_t Left_Encoder_Interrupt_flag;

//speeds
volatile float Right_Angular_Velocity;
volatile float Right_RPM;
volatile int16_t Right_Direction;

volatile float Left_Angular_Velocity;
volatile float Left_RPM;
volatile int16_t Left_Direction;

//distances
//volatile int32_t leftTotal;
//volatile int32_t rightTotal;
//volatile int32_t fwdTotal;
//volatile int32_t rotTotal;

// local variables
static  volatile int16_t oldRightEncoder;
static  volatile int16_t rightEncoder;
static  volatile int32_t rightEncoder_Diff;

static  volatile int16_t oldLeftEncoder;
static  volatile int16_t leftEncoder;
static  volatile int32_t leftEncoder_Diff;
/* Functions ---------------------------------------------------------------- */
/*******************************************************************************
* Function Name  : --Encoders_Init
* Description    : --Set Time4 and Timer3 as encoder interface mode
									 --TIM4_CH1-->PB6-->AVout
									 --TIM4_CH2-->PB7-->BVout
									 --TIM3_CH1-->PC6-->AVout
									 --TIM3_CH2-->PC7-->BVout
									 --
* Input          : --NULL
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/

void Encoders_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure2;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure2;
	
  // set clocks for each of the ports
  RCC_AHB1PeriphClockCmd (Right_EncoderA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (Right_EncoderB_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd (Left_EncoderA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (Left_EncoderB_GPIO_CLK, ENABLE);
	// Timer peripheral clock enable
  RCC_APB1PeriphClockCmd (Right_Encoder_TIMER_CLK, ENABLE);
	RCC_APB1PeriphClockCmd (Left_Encoder_TIMER_CLK, ENABLE);

  // set every inputs mode with pullups for Timer 4
  GPIO_StructInit (&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = Right_EncoderA_PIN | Right_EncoderB_PIN;
  GPIO_Init (Right_EncoderB_GPIO_PORT, &GPIO_InitStructure);
	
	// set every inputs mode with pullups for Timer 3
  GPIO_StructInit (&GPIO_InitStructure2);
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure2.GPIO_Pin = Left_EncoderA_PIN | Left_EncoderB_PIN;
  GPIO_Init (Left_EncoderB_GPIO_PORT, &GPIO_InitStructure2);

  // Connect the pins to their Alternate Functions
  GPIO_PinAFConfig (Right_EncoderA_GPIO_PORT, Right_EncoderA_SOURCE, Right_EncoderA_AF);
  GPIO_PinAFConfig (Right_EncoderB_GPIO_PORT, Right_EncoderB_SOURCE, Right_EncoderB_AF);
	// Connect the pins to their Alternate Functions
	GPIO_PinAFConfig (Left_EncoderA_GPIO_PORT, Left_EncoderA_SOURCE, Left_EncoderA_AF);
  GPIO_PinAFConfig (Left_EncoderB_GPIO_PORT, Left_EncoderB_SOURCE, Left_EncoderB_AF);

  // set time encoder inputs
  // set both inputs to rising polarity to let it use both edges
  TIM_EncoderInterfaceConfig (Right_Encoder_TIMER, TIM_EncoderMode_TI12, 
                              TIM_ICPolarity_Rising, 
                              TIM_ICPolarity_Rising);
	// set both inputs to rising polarity to let it use both edges
  TIM_EncoderInterfaceConfig (Left_Encoder_TIMER, TIM_EncoderMode_TI12, 
                              TIM_ICPolarity_Rising, 
                              TIM_ICPolarity_Rising);														
															
	//set the auto reload value to (65535-1)/2 to make sure get positive number everytime													
  TIM_SetAutoreload (Right_Encoder_TIMER, 0x7fff);
	
	//set the auto reload value to (65535-1)/2 to make sure get positive number everytime													
  TIM_SetAutoreload (Left_Encoder_TIMER, 0x7fff);
	
	//enable timer4 interrupt
  TIM_ITConfig(Right_Encoder_TIMER,TIM_IT_Update,ENABLE); 
	
	//enable timer3 interrupt
  TIM_ITConfig(Left_Encoder_TIMER,TIM_IT_Update,ENABLE); 

	//init the interrupt priority
	NVIC_InitStructure.NVIC_IRQChannel = Right_Encoder_TIMWE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Init(&NVIC_InitStructure);
	
	//init the interrupt priority
	NVIC_InitStructure2.NVIC_IRQChannel = Left_Encoder_TIMWE_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Init(&NVIC_InitStructure2);

  //start the timer/counters
  TIM_Cmd (Right_Encoder_TIMER, ENABLE);
	TIM_Cmd (Left_Encoder_TIMER, ENABLE);
	
  Encoders_Reset();
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
void Encoders_Reset (void)
{
  __disable_irq();
	
  oldRightEncoder = 0;
  rightEncoder = 0;
	rightEncoder_Diff = 0;
	Right_Encoder_Interrupt_flag = 0;
	Right_Angular_Velocity = 0;
	Right_Direction = 0;
	Right_RPM = 0;
	
	oldLeftEncoder = 0;
  leftEncoder = 0;
	leftEncoder_Diff = 0;
	Left_Encoder_Interrupt_flag = 0;
	Left_Angular_Velocity = 0;
	Left_Direction = 0;
	Left_RPM = 0;
	
  TIM_SetCounter (Right_Encoder_TIMER, 0);
	TIM_SetCounter (Left_Encoder_TIMER, 0);
  Encoders_Read();
	
  __enable_irq();
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
void Encoders_Read (void)
{
	//Right motor
	oldRightEncoder = rightEncoder;
  rightEncoder = TIM_GetCounter (Right_Encoder_TIMER);
	
	if(rightEncoder>oldRightEncoder)
	{
		if(Right_Encoder_Interrupt_flag == 1)//interrupt occurs
		{
			Right_Encoder_Interrupt_flag = 0;
			Right_Direction = -1;//record the direction
			rightEncoder_Diff = rightEncoder - 32767 - oldRightEncoder;
		}
		else
		{
			Right_Direction = 1;//record the direction
			rightEncoder_Diff = rightEncoder - oldRightEncoder;
		}
	}
	else if(rightEncoder<oldRightEncoder)
	{
		if(Right_Encoder_Interrupt_flag == 1)//interrupt occurs
		{
			Right_Encoder_Interrupt_flag = 0;
			Right_Direction = 1;//record the direction
			rightEncoder_Diff = rightEncoder + 32767 - oldRightEncoder;
		}
		else
		{
			Right_Direction = -1;//record the direction
			rightEncoder_Diff = rightEncoder - oldRightEncoder;
		}
	}
	else//speed = 0
	{
		rightEncoder_Diff = 0;
	}
	
	//Left Motor
	oldLeftEncoder = leftEncoder;
  leftEncoder = TIM_GetCounter (Left_Encoder_TIMER);
	
	if(leftEncoder>oldLeftEncoder)
	{
		if(Left_Encoder_Interrupt_flag == 1)//interrupt occurs
		{
			Left_Encoder_Interrupt_flag = 0;
			Left_Direction = -1;//record the direction
			leftEncoder_Diff = leftEncoder - 32767 - oldLeftEncoder;
		}
		else
		{
			Left_Direction = 1;//record the direction
			leftEncoder_Diff = leftEncoder - oldLeftEncoder;
		}
	}
	else if(leftEncoder<oldLeftEncoder)
	{
		if(Left_Encoder_Interrupt_flag == 1)//interrupt occurs
		{
			Left_Encoder_Interrupt_flag = 0;
			Left_Direction = 1;//record the direction
			leftEncoder_Diff = leftEncoder + 32767 - oldLeftEncoder;
		}
		else
		{
			Left_Direction = -1;//record the direction
			leftEncoder_Diff = leftEncoder - oldLeftEncoder;
		}
	}
	else//speed = 0
	{
		leftEncoder_Diff = 0;
	}
	
	//get the angular velocity (rad/s)
	Right_Angular_Velocity = ((2*3.14*((float)rightEncoder_Diff/(4*Pulse_perCiecle)))/(float)SysTick_Interval)*1000;
	Left_Angular_Velocity = ((2*3.14*((float)leftEncoder_Diff/(4*Pulse_perCiecle)))/(float)SysTick_Interval)*1000;
	
	//get the rotate speed (r/min)
	Right_RPM = (60*Right_Angular_Velocity)/(2*PI);
	Left_RPM = (60*Left_Angular_Velocity)/(2*PI);
	
	//get the distance
	
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
void TIM4_IRQHandler(void)
{  
	if(TIM_GetITStatus(Right_Encoder_TIMER,TIM_IT_Update)==SET) //溢出中断
	{
		//record interrupt
		Right_Encoder_Interrupt_flag = 1;

	}
	TIM_ClearITPendingBit(Right_Encoder_TIMER,TIM_IT_Update);  //清除中断标志位
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
void TIM3_IRQHandler(void)
{  
	if(TIM_GetITStatus(Left_Encoder_TIMER,TIM_IT_Update)==SET) //溢出中断
	{
		//record interrupt
		Left_Encoder_Interrupt_flag = 1;

	}
	TIM_ClearITPendingBit(Left_Encoder_TIMER,TIM_IT_Update);  //清除中断标志位
}

