#include "stm32f10x.h"
#include "Motor.h"
#include "Encoder.h"

typedef struct {
    float Target_Speed;
    float Actual_Speed;
    float Err;
    float Err_Last;
    float Err_Last2;
    float Kp, Ki, Kd;
    float Out;
} PID_TypeDef;

PID_TypeDef PID_Left, PID_Right;

float Target_Left = 0, Target_Right = 0;

void PID_Init(void)
{
    PID_Left.Kp = 15.0f;  PID_Left.Ki = 1.0f;  PID_Left.Kd = 0.5f;
    PID_Right.Kp = 15.0f; PID_Right.Ki = 1.0f; PID_Right.Kd = 0.5f;
    
    PID_Left.Err_Last = 0.0f;  PID_Left.Err_Last2 = 0.0f;
    PID_Right.Err_Last = 0.0f; PID_Right.Err_Last2 = 0.0f;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    TIM_InternalClockConfig(TIM1);
	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;       
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;   
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;         
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM1, ENABLE);
}

int16_t PID_Calculate(PID_TypeDef *pid, float target, float actual)
{
    pid->Target_Speed = target;
    pid->Actual_Speed = actual;
    pid->Err = pid->Target_Speed - pid->Actual_Speed;
    
    float Out = pid->Kp * (pid->Err - pid->Err_Last) + 
                      pid->Ki * pid->Err + 
                      pid->Kd * (pid->Err - 2.0f * pid->Err_Last + pid->Err_Last2);
		
    pid->Out += Out;
		pid->Err_Last2 = pid->Err_Last;
    pid->Err_Last = pid->Err;
	
    if(pid->Out > 100.0f)  pid->Out = 100.0f;
    if(pid->Out < -100.0f) pid->Out = -100.0f;
    
    return (int16_t)pid->Out;
}

void PID_Reset(void)
{
   __disable_irq(); 
    
    PID_Left.Err_Last = 0.0f;
    PID_Left.Err_Last2 = 0.0f;
    PID_Left.Out = 0.0f;      
    
    PID_Right.Err_Last = 0.0f;
    PID_Right.Err_Last2 = 0.0f;
    PID_Right.Out = 0.0f;     
    
    __enable_irq();  
}
