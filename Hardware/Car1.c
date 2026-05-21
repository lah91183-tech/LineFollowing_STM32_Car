#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Delay.h"
#include "Car1.h"
#include "PIDcontrol.h"

extern float Target_Left, Target_Right;
static uint8_t Car_State = 0;

void Car_Init(void){
	Motor_Init();
}

void Car_Stop(void){
	if (Car_State != 0) {
		Target_Left = 0.0f;
		Target_Right = 0.0f;
		PID_Reset();
		Car_State = 0; 
	}
}

void Go_Ahead(void){
	if (Car_State != 1) {
		Target_Left = 25.0f;   
		Target_Right = 25.0f;
		PID_Reset();
		Car_State = 1; 
	}
}

void Go_Back(void){
	if (Car_State != 2) {
		Target_Left = -25.0f;
		Target_Right = -25.0f;
		PID_Reset();
		Car_State = 2;
	}
}

void Self_Left(void){
	if (Car_State != 3) {
		Target_Left = -15.0f;  
		Target_Right = 15.0f;
		PID_Reset();
		Car_State = 3;
	}
}

void Self_Right(void){
	if (Car_State != 4) {
		Target_Left = 15.0f;   
		Target_Right = -15.0f;
		PID_Reset();
		Car_State = 4;
	}
}

void Turn_Left(void){
	if (Car_State != 5) {
		Target_Left = 0.0f;  
		Target_Right = 15.0f;
		PID_Reset();
		Car_State = 5;
	}
}

void Turn_Right(void){
	if (Car_State != 6) {
		Target_Left = 15.0f;   
		Target_Right = 0.0f;
		PID_Reset();
		Car_State = 6;
	}
}






