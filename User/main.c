#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor.h"
#include "Car1.h"
#include "Serial.h"
#include "Encoder.h"
#include "PIDcontrol.h"
#include "PWMServo.h"
#include "Servo.h"
#include "Ultersound.h"
#include <string.h>
#include <stdio.h>

extern PID_TypeDef PID_Left, PID_Right;
extern float Target_Left, Target_Right;
int16_t Data1;

extern volatile uint8_t  Echo_State;   
extern volatile uint16_t High_Time;     
extern volatile uint8_t  Distance_Ready;

int main(void)
{
    Motor_Init();
    Encoder_Init_Left();    
    Encoder_Init_Right();   
    Serial_Init();
    Car_Init();
    
    PWM_Init2();
    Ultrasound_Init();
    Servo_Init();
    PID_Init();            
  
	
    float distance = 0.0f;
    uint32_t ultrasound_count = 0;
	
    Serial_Printf("System Ready! Use 0x37/0x38 to tune Kp.\r\n");
	
    while(1)
    {
        ultrasound_count++;
        if(ultrasound_count >= 400000) 
        {
            Ultrasound_Trigger(); 
            ultrasound_count = 0;
        }
		
        distance = Ultrasound_GetDistance();
		

        if(distance < 20.0f)
        {
            Car_Stop(); 
        }
        Serial_SendNumber(distance, 3);
    }
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        Data1 = USART_ReceiveData(USART1);
        switch(Data1)
        {
            case 0x30 : Car_Stop();         break; 
            case 0x31 : Go_Ahead();         break;
            case 0x32 : Go_Back();          break;
            case 0x33 : Turn_Left();        break;
            case 0x34 : Turn_Right();       break;
            case 0x35 : Turn_Back_Left();   break;
            case 0x36 : Turn_Back_Right();  break;
			
          
            /*
            case 0x37 : 
                PID_Left.Kp += 1.0f;  
                PID_Right.Kp += 1.0f;
                Serial_Printf("Kp Up: %.1f\r\n", PID_Left.Kp);
                break;
                
            case 0x38 : 
                if(PID_Left.Kp > 0) { 
                    PID_Left.Kp -= 1.0f; 
                    PID_Right.Kp -= 1.0f;
                }
                Serial_Printf("Kp Down: %.1f\r\n", PID_Left.Kp);
                break;
                
            case 0x39 : 
                PID_Left.Ki += 0.2f;  
                PID_Right.Ki += 0.2f;
                Serial_Printf("Ki Up: %.1f\r\n", PID_Left.Ki);
                break;
			
            case 0x3A : 
                if(PID_Left.Ki >= 0.2){
                    PID_Left.Ki -= 0.2f;  
                    PID_Right.Ki -= 0.2f;
                    Serial_Printf("Ki Down: %.1f\r\n", PID_Left.Ki);
                }
                break;

            case 0x3B : 
                if(PID_Left.Kd >= 0.2){
                    PID_Left.Kd -= 0.2f;  
                    PID_Right.Kd -= 0.2f;
                    Serial_Printf("Kd Down: %.1f\r\n", PID_Left.Ki);
                }
                break;
			
            case 0x3C : 
                PID_Left.Kd += 0.2f;  
                PID_Right.Kd += 0.2f;
                Serial_Printf("Kd Up: %.1f\r\n", PID_Left.Ki);
                break;
            */
				
            case 0x3D : Servo_SetAngle(0);   break;
            case 0x3E : Servo_SetAngle(90);  break;
            case 0x3F : Servo_SetAngle(180); break;
				
            default: break;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        int16_t Actual_Left = Encoder_GetLeft();
        int16_t Actual_Right = Encoder_GetRight();

        int16_t Output_Left = PID_Calculate(&PID_Left, Target_Left, Actual_Left);
        int16_t Output_Right = PID_Calculate(&PID_Right, Target_Right, Actual_Right);

        Motor_SetLeftSpeed(Output_Left);
        Motor_SetRightSpeed(Output_Right);

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
    {
        if (Echo_State == 0) 
        {
            TIM_SetCounter(TIM2, 0); 
            TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling); 
            Echo_State = 1;      
        }
        else if (Echo_State == 1)
        {
            High_Time = TIM_GetCapture3(TIM2); 
            TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising);  
            Echo_State = 0;           
            Distance_Ready = 1;     
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3); 
    }
}
