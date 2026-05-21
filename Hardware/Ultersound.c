#include "stm32f10x.h"                  // Device header
#include "Ultersound.h"
#include "Delay.h"

uint8_t  Echo_State = 0;    
uint16_t High_Time = 0;      
volatile uint8_t  Distance_Ready = 0; 
static float Real_Distance = 0.0f;          

void Ultrasound_Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;             
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;       
  TIM_ICInitStructure.TIM_ICFilter = 0x0F;                     
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}

void Ultrasound_Trigger(void){
    if (Echo_State == 0) 
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12); 
        
        for(volatile uint32_t i = 0; i < 50; i++); 
        
        GPIO_ResetBits(GPIOB, GPIO_Pin_12); 
    }
}

float Ultrasound_GetDistance(void){
    if (Distance_Ready == 1) 
    {
        Real_Distance = High_Time * 0.017f;
        Distance_Ready = 0;  
    }
    return Real_Distance;    
}

//void TIM4_IRQHandler(void)//ultrasound
//{
//    if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
//    {
//        if (Echo_State == 0) 
//        {
//            TIM_SetCounter(TIM4, 0); 
//            TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling); 
//            Echo_State = 1;      
//        }
//        else if (Echo_State == 1)
//        {
//            High_Time = TIM_GetCapture3(TIM4); 
//            TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising);  
//            Echo_State = 0;           
//            Distance_Ready = 1;     
//        }
//        
//        TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); 
//    }
//}



