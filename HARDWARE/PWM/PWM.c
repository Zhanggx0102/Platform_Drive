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

#include "PWM.h"
#include "PID_Controller.h"

/* Variable ----------------------------------------------------------------- */
extern volatile PID_Increment_Struct Right_Motor_PID;
extern volatile PID_Increment_Struct Left_Motor_PID;

/*******************************************************************************
* Function Name  : --TIM10_PWM_Init
* Description    : --arr :auto reload value
									 --psc :clock divide factor

* Input          : --arr 
									 --psc
* Output         : --NULL
* Return         : --NULL
*typical applacation: 
									 --TIM10_PWM_Init(1000-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��,����װ��ֵ500��PWMƵ��Ϊ 1M/500=2Khz.    
*******************************************************************************/
void TIM10_PWM_Init(u32 arr,u32 psc)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	//TIM10ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//ʹ��GPIOFʱ��	
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10); //GPIOF6����Ϊ��ʱ��10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOF6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOF,&GPIO_InitStructure);              //��ʼ��PF6
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseStructure);//��ʼ����ʱ��10
	
	//Init TIM10 Channel1 PWM Mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //ʹ��TIM10��CCR1�ϵ�Ԥװ�ؼĴ���
 
  //Enable ARPE 
  TIM_ARRPreloadConfig(TIM10,ENABLE);
	
	//Enable TIM10
	TIM_Cmd(TIM10, ENABLE); 
}

/*******************************************************************************
* Function Name  : --TIM10_PWM_SetDuty
* Description    : --ratio :rang(0~2100)(because arr=2100 in main.c declared: TIM10_PWM_Init(2100-1,4-1);)
									 --
									 --
									 --
* Input          : --ratio
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/
void TIM10_PWM_SetDuty(u32 ratio)
{
	TIM_SetCompare1(TIM10,ratio);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}

/*******************************************************************************
* Function Name  : --TIM11_PWM_Init
* Description    : --arr :auto reload value
									 --psc :clock divide factor

* Input          : --arr 
									 --psc
* Output         : --NULL
* Return         : --NULL
*typical applacation: 
									 --TIM11_PWM_Init(1000-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��,����װ��ֵ500��PWMƵ��Ϊ 1M/500=2Khz.    
*******************************************************************************/
void TIM11_PWM_Init(u32 arr,u32 psc)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);  	//TIM11ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//ʹ��GPIOFʱ��	
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_TIM11); //GPIOF7����Ϊ��ʱ��11
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;           //GPIOF6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOF,&GPIO_InitStructure);              //��ʼ��PF7
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseStructure);//��ʼ����ʱ��10
	
	//Init TIM11 Channel1 PWM Mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM11OC1
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);  //ʹ��TIM11��CCR1�ϵ�Ԥװ�ؼĴ���
 
  //Enable ARPE 
  TIM_ARRPreloadConfig(TIM11,ENABLE);
	
	//Enable TIM10
	TIM_Cmd(TIM11, ENABLE); 
}

/*******************************************************************************
* Function Name  : --TIM11_PWM_SetDuty
* Description    : --ratio :rang(0~2100)(because arr=2100 in main.c declared: TIM11_PWM_Init(2100-1,4-1);)
									 --
									 --
									 --
* Input          : --ratio
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/
void TIM11_PWM_SetDuty(u32 ratio)
{
	TIM_SetCompare1(TIM11,ratio);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}

/*******************************************************************************
* Function Name  : --Motor_Control_Init
* Description    : --
									 --
									 --
									 --
* Input          : --NULL
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/
void Motor_Control_Init()
{
	GPIO_InitTypeDef GPIO_Init_Motor_Control;
	GPIO_InitTypeDef GPIO_Init_Motor_Control2;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIODʱ��
	
	GPIO_Init_Motor_Control.GPIO_Mode=GPIO_Mode_OUT;//typical out mode
	GPIO_Init_Motor_Control.GPIO_OType=GPIO_OType_PP;//push-pull output mode
	GPIO_Init_Motor_Control.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//pin
	GPIO_Init_Motor_Control.GPIO_PuPd=GPIO_PuPd_UP;//Pull-Up
	GPIO_Init_Motor_Control.GPIO_Speed=GPIO_Speed_100MHz;	
	GPIO_Init(GPIOD, &GPIO_Init_Motor_Control);
	
	GPIO_Init_Motor_Control2.GPIO_Mode = GPIO_Mode_OUT;//typical out mode
	GPIO_Init_Motor_Control2.GPIO_OType = GPIO_OType_PP;// push-pull output mode
	GPIO_Init_Motor_Control2.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;//pin
	GPIO_Init_Motor_Control2.GPIO_PuPd = GPIO_PuPd_UP;//Pull-Up
	GPIO_Init_Motor_Control2.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_Init(GPIOE, &GPIO_Init_Motor_Control2);
	
	GPIO_SetBits(GPIOD, GPIO_Pin_6|GPIO_Pin_7);//both pin high level , the motor in  suspension mode
	GPIO_SetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_6);//both pin high level , the motor in  suspension mode
}

/*******************************************************************************
* Function Name  : --Motor_Control
* Description    : -- four mode
									 --
									 --
									 --
* Input          : --Hanger        0
									 --Brake         1
									 --Foreward_PWM  2
									 --Reversal_PWM  3
* Output         : --NULL
* Return         : --suces >0
									 --failed:<0
*******************************************************************************/
int Motor_State_Control(u8 signal_right, u8 signal_left)
{
	int flag = 0;
	//right motor
	switch(signal_right)
	{
		case 0://Dangling mode
			GPIO_SetBits(GPIOD, GPIO_Pin_6|GPIO_Pin_7);
		  flag = 1;
			break;
		case 1://Brake mode
			GPIO_ResetBits(GPIOD,GPIO_Pin_6|GPIO_Pin_7);
			flag = 1;
			break;
		case 2://Foreward mode
		  GPIO_SetBits(GPIOD, GPIO_Pin_7);
			GPIO_ResetBits(GPIOD,GPIO_Pin_6);
			flag = 1;
			break;
		case 3://Reversal mode
			GPIO_SetBits(GPIOD, GPIO_Pin_6);
			GPIO_ResetBits(GPIOD,GPIO_Pin_7);
		  flag = 1;
			break;
		default:
			flag = -1;
			break;
	}
	//left motor
	switch(signal_left)
	{
		case 0://Dangling mode
			GPIO_SetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_6);
		  flag = 1;
			break;
		case 1://Brake mode
			GPIO_ResetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_6);
			flag = 1;
			break;
		case 2://Foreward mode
		  GPIO_SetBits(GPIOE, GPIO_Pin_5);
			GPIO_ResetBits(GPIOE,GPIO_Pin_6);
			flag = 1;
			break;
		case 3://Reversal mode
			GPIO_SetBits(GPIOE, GPIO_Pin_6);
			GPIO_ResetBits(GPIOE,GPIO_Pin_5);
		  flag = 1;
			break;
		default:
			flag = -1;
			break;
	}
	
	return flag;
}
/*******************************************************************************
* Function Name  : --Motor_PWM_Creater
* Description    : --
									 --
									 --
									 --
* Input          : --NULL
* Output         : --NULL
* Return         : --NULL
*******************************************************************************/
void Motor_PWM_Creater (float Target_Velocity_R, u8 signal_R, float Target_Velocity_L, u8 signal_L)
{
	u32 PWM_Ratio_R;
	u32 PWM_Ratio_L;
	
	PWM_Ratio_R = (int)((0.789474*Target_Velocity_R+17)*21);
	Right_Motor_PID.PWM_Out = (float)PWM_Ratio_R;

	PWM_Ratio_L = (int)((0.789474*Target_Velocity_L+17)*21);
	Left_Motor_PID.PWM_Out = (float)PWM_Ratio_L;
	
	TIM10_PWM_SetDuty(PWM_Ratio_R);
	TIM11_PWM_SetDuty(PWM_Ratio_L);
	
	//Set the direction
	Motor_State_Control(signal_R,signal_L);
	
}


