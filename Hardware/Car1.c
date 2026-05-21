#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Delay.h"
#include "Car1.h"

extern float Target_Left, Target_Right;

void Car_Init(void){
	Motor_Init();
}

void Go_Ahead(void){
	Target_Left = 25.0f;   
    Target_Right = 25.0f;
}

void Go_Back(void){
	Target_Left = -25.0f;
    Target_Right = -25.0f;
}

void Turn_Left(void){
	Target_Left = -15.0f;  
    Target_Right = 15.0f;
}

void Turn_Right(void){
	Target_Left = 15.0f;   
    Target_Right = -15.0f;
}

void Turn_Back_Left(void){
	Target_Left = -10.0f;
    Target_Right = -25.0f;
}

void Turn_Back_Right(void){
	Target_Left = -25.0f;
    Target_Right = -10.0f;
}

void Car_Stop(void){
	Target_Left = 0.0f;
    Target_Right = 0.0f;
}



