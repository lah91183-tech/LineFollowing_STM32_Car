#include "stm32f10x.h"                  // Device header
#include "Infrared.h"
#include "Car1.h"

void Infrared_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Infrared_Track(void){
	static uint8_t Last_Direction = 0;
	
	uint8_t L2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);  
  uint8_t L1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13); 
  uint8_t R1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14); 
  uint8_t R2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15); 
	
	uint8_t sensor_state = (L2 << 3) | (L1 << 2) | (R1 << 1) | R2;
	
	switch (sensor_state)
  {
      case 0x00: 
          if (Last_Direction == 1)      { Self_Left(); }  
          else if (Last_Direction == 2) { Self_Right(); }
          else                          { Go_Ahead(); }   
          break;
            
      case 0x0F: 
          Car_Stop();
          Last_Direction = 0;
          break;
            
      case 0x03: 
      case 0x01: 
          Self_Right();       
          Last_Direction = 2;
          break;
            
      case 0x02: 
          Turn_Right();       
          Last_Direction = 2; 
          break;
            
      case 0x0C: 
      case 0x08: 
          Self_Left();       
          Last_Direction = 1; 
          break;
            
      case 0x04: 
          Turn_Left();        
          Last_Direction = 1;
          break;
            
      case 0x06: 
          Go_Ahead();
          Last_Direction = 0; 
          break;
				
		default:
          break;
	
			}
}
