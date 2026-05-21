#ifndef __PIDCONTROL_H
#define __PIDCONTROL_H

typedef struct {
    float Target_Speed;  
    float Actual_Speed;  
    float Err;         
    float Err_Last;   
    float Integral;     
    float Kp, Ki, Kd;    
    float Voltage;     
} PID_TypeDef;


extern PID_TypeDef PID_Left, PID_Right;
extern float Target_Left, Target_Right;

void PID_Init(void);
int16_t PID_Calculate(PID_TypeDef *pid, float target, float actual);
void ControlTimer_Init(void);

#endif

