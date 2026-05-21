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
#include "Infrared.h"

extern PID_TypeDef PID_Left, PID_Right;
extern float Target_Left, Target_Right;
int16_t Data1;

extern volatile uint8_t  Echo_State;   
extern volatile uint16_t High_Time;     
extern volatile uint8_t  Distance_Ready;

void Car_Avoidance_Logic(void);

int main(void)
{
    Motor_Init();
    Encoder_Init_Left();    
    Encoder_Init_Right();   
    Serial_Init();
    Car_Init();
		Infrared_Init();
    
    PWM_Init2();
    Ultrasound_Init();
    Servo_Init();
    PID_Init();            

    Serial_Printf("System Ready! Use 0x37/0x38 to tune Kp.\r\n");
	
    while(1)
    {
//      Car_Avoidance_Logic();
			
			Infrared_Track();
     
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
            case 0x33 : Self_Left();        break;
            case 0x34 : Self_Right();       break;
            case 0x35 : Self_Left();   			break;
            case 0x36 : Self_Right(); 			break;
						
			
            /*
            case 0x40 : 
                PID_Left.Kp += 1.0f;  
                PID_Right.Kp += 1.0f;
                Serial_Printf("Kp Up: %.1f\r\n", PID_Left.Kp);
                break;
                
            case 0x41 : 
                if(PID_Left.Kp > 0) { 
                    PID_Left.Kp -= 1.0f; 
                    PID_Right.Kp -= 1.0f;
                }
                Serial_Printf("Kp Down: %.1f\r\n", PID_Left.Kp);
                break;
                
            case 0x42 : 
                PID_Left.Ki += 0.2f;  
                PID_Right.Ki += 0.2f;
                Serial_Printf("Ki Up: %.1f\r\n", PID_Left.Ki);
                break;
			
            case 0x43 : 
                if(PID_Left.Ki >= 0.2){
                    PID_Left.Ki -= 0.2f;  
                    PID_Right.Ki -= 0.2f;
                    Serial_Printf("Ki Down: %.1f\r\n", PID_Left.Ki);
                }
                break;

            case 0x44 : 
                if(PID_Left.Kd >= 0.2){
                    PID_Left.Kd -= 0.2f;  
                    PID_Right.Kd -= 0.2f;
                    Serial_Printf("Kd Down: %.1f\r\n", PID_Left.Ki);
                }
                break;
			
            case 0x45 : 
                PID_Left.Kd += 0.2f;  
                PID_Right.Kd += 0.2f;
                Serial_Printf("Kd Up: %.1f\r\n", PID_Left.Ki);
                break;
            */
						
						case 0x50 : Servo_SetAngle(0);   break;
            case 0x51 : Servo_SetAngle(90);  break;
            case 0x52 : Servo_SetAngle(180); break;
           
				
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

void Car_Avoidance_Logic(void){
	float distance = 0.0f;
  static uint32_t ultrasound_count = 0;
	ultrasound_count++;
        if(ultrasound_count >= 400000) 
        {
            Ultrasound_Trigger(); 
            ultrasound_count = 0;
        }
		
        distance = Ultrasound_GetDistance();
		    Serial_SendNumber(distance, 3);

        if(distance < 10.0f)
        {
          Car_Stop(); 
					Servo_SetAngle(0);
					Delay_ms(600);
					Ultrasound_Trigger();
					Delay_ms(40);
					uint16_t RightDistance = Ultrasound_GetDistance();
					Serial_SendNumber(RightDistance, 3);
					
					if(RightDistance > 10.0f){
						Servo_SetAngle(90);
						Delay_ms(600);
						Self_Right();
						Delay_ms(1000);
						Go_Ahead();			
					}
					else{
						Servo_SetAngle(180);
						Delay_ms(600);
						
						Ultrasound_Trigger();
						Delay_ms(40);
						uint16_t LeftDistance = Ultrasound_GetDistance();
						Serial_SendNumber(LeftDistance, 3);
						
						if(LeftDistance > 10.0f){
							Servo_SetAngle(90);
							Delay_ms(600);
							Self_Left();
							Delay_ms(1000);
							Go_Ahead();			
						}
						else{
							Servo_SetAngle(90);
							Self_Left();
							Delay_ms(2000);
							Go_Ahead();
						}
					}
        }
	
}
