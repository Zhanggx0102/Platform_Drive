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
#include "PID_Controller.h"
#include "PWM.h"
/* Definition ----------------------------------------------------------------- */


/* Variable ----------------------------------------------------------------- */
volatile PID_Increment_Struct Right_Motor_PID;

volatile float a , b;//debug

/* Functions ---------------------------------------------------------------- */
void Increment_PID_Mode(volatile PID_Increment_Struct* Increment_PID)
{
	float Err, dErr, ddErr, dErrP, dErrI, dErrD;
	
	Err = Increment_PID->Now_err;
	dErr = Increment_PID->Now_err - Increment_PID->Pre_err;
	ddErr = Increment_PID->Pre_err - Increment_PID->Pre_Pre_err;
	//update the errors
	Increment_PID->Pre_Pre_err = Increment_PID->Pre_err;
	Increment_PID->Pre_err = Increment_PID->Now_err;
	//debug
	a = Err;
	//b = dErr;
	
	dErrP = dErr;
	dErrI = Err;
	dErrD = Err - 2*dErr + ddErr;


	Increment_PID->PWM_Increment_Out = Increment_PID->Kp*dErrP + Increment_PID->Ki*dErrI + Increment_PID->Kd*dErrD;
	//smooth the PWM_Increment_Out
	if(Increment_PID->PWM_Increment_Out <= -40)
	{
		Increment_PID->PWM_Increment_Out = -40;
	}
	b = Increment_PID->PWM_Increment_Out;
	
	Increment_PID->PWM_Out = Increment_PID->PWM_Out + Increment_PID->PWM_Increment_Out;
	//limit the output rang
	if(Increment_PID->PWM_Out>2100)
	{
		Increment_PID->PWM_Out = 2100;
	}
	else if(Increment_PID->PWM_Out < 0)
	{
		Increment_PID->PWM_Out = 0;
	}

	//set the new PWM duty
	TIM10_PWM_SetDuty((int)Increment_PID->PWM_Out);
	
}

void PID_Controller(float Target_Velocity, float Actual_Velocity)
{ 
	Right_Motor_PID.Now_err = Target_Velocity - Actual_Velocity;
	
	Right_Motor_PID.Kp = 5;
	Right_Motor_PID.Ki = 1;
	Right_Motor_PID.Kd = 1;
	
	Increment_PID_Mode(&Right_Motor_PID);
}





