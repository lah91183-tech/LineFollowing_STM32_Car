#include "stm32f10x.h"
#include "Motor.h"
#include "Encoder.h"

typedef struct{
	float Target_Speed;
	float Actual_Speed;
	float Err;
	float Err_Last;
	float Integral;
	float Kp, Ki, Kd;
	float Out;
} PID_TypeDef;

PID_TypeDef PID_Left, PID_Right;

float Target_Left = 0, Target_Right = 0;

void PID_Init(void){
		PID_Left.Kp = 15.0;  PID_Left.Ki = 1.0;  PID_Left.Kd = 0.5;
    PID_Right.Kp = 15.0; PID_Right.Ki = 1.0; PID_Right.Kd = 0.5;
    
    PID_Left.Integral = 0;  PID_Left.Err_Last = 0;
    PID_Right.Integral = 0; PID_Right.Err_Last = 0;
	
}

int16_t PID_Calculate(PID_TypeDef *pid, float target, float actual){
	pid->Target_Speed = target;
	pid->Actual_Speed = actual;
	pid->Err = pid->Target_Speed - pid->Actual_Speed;
	pid->Integral += pid->Err;
	
	if(pid->Integral > 2000) {pid->Integral = 2000;}
	if(pid->Integral < -2000) {pid->Integral = -2000;}
	
	pid->Out = pid->Kp * pid->Err + pid->Ki * pid->Integral + pid->Kd * (pid->Err - pid->Err_Last);
  pid->Err_Last = pid->Err;
	
	if(pid->Out > 100)  pid->Out = 100;
  if(pid->Out < -100) pid->Out = -100;
    
  return (int16_t)pid->Out;
	
}


