#include "T100MotorControl.h"
#include "Initialize.h"


/*
 //////////////////// T100 Motor Control Functions //////////////////////////
 *
 */


//PWM control

void Initialize_Motors(){


	INIT_TIM_PWM(TIM5, PWM_PERIOD,PrescalerRegister);
	INIT_TIM_PWM(TIM1,PWM_PERIOD,PrescalerRegister);
	INIT_TIM_PWM(TIM11,PWM_PERIOD,PrescalerRegister);

	HAL_Delay(100);
	//start pwm 1
	Start_PWM(TIM5,2,INITIALIZE_DUTY,GPIOC,10,2);
	HAL_Delay(1);
	//start pwm 2
	Start_PWM(TIM1,4,INITIALIZE_DUTY,GPIOA,11,1);
	HAL_Delay(1);
	//start pwm 3
	Start_PWM(TIM1,1,INITIALIZE_DUTY,GPIOA,8,1);
	HAL_Delay(1);
	//start pwm 4
	Start_PWM(TIM5,1,INITIALIZE_DUTY,GPIOB,12,2);
	HAL_Delay(1);
	//start pwm 5
	Start_PWM(TIM5,3,INITIALIZE_DUTY,GPIOC,11,2);
	HAL_Delay(1);
	//start pwm 6
	Start_PWM(TIM11,1,INITIALIZE_DUTY,GPIOC,12,3);
	HAL_Delay(1);
	//start pwm 7
	Start_PWM(TIM5,4,INITIALIZE_DUTY,GPIOB,11,2);
	HAL_Delay(1);
	//start pwm 8
	Start_PWM(TIM11,1,INITIALIZE_DUTY,GPIOB,9,3);
}


/*
Bitwise_Motor_Control function sets speed/direction of multiple motors.

INPUTS:

"MotorSelect" input 1-255 where each motor is adjusted based on the bitwise position of the 1's
		ie: Motor Select input of 4 = 00000100 -> motor 3 will be affected
			Motor Select input of 127 = 01111111 -> motors 1-7 will be affected

"Speed" input -100 to 100 for reverse/forward speed respectively

//Returns 0 if successful
//Returns -1 for invalid speed input
//Returns -2 for invalid motor input

 */
int Bitwise_Motor_Control(vu8 MotorSelect,float Speed){

	const int SpeedRange = 100;


	if (Speed > 100 || Speed < -100){
		return -1;
	}

	if(MotorSelect > 255 || MotorSelect < 0){
		return -2;
	}

	Speed = (Speed/SpeedRange)*MOTOR_RANGE;
	float Duty = (1500 + Speed)/PWM_PERIOD;
	Duty *= 100;

	if(MotorSelect & 1){
		Set_Duty_Cycle(TIM5,2,Duty); //Motor1 ...
	}
	else if(MotorSelect & 2){
		Set_Duty_Cycle(TIM1,4,Duty);
	}
	else if(MotorSelect & 4){
		Set_Duty_Cycle(TIM1,1,Duty);
	}
	else if(MotorSelect & 8){
		Set_Duty_Cycle(TIM5,1,Duty);
	}
	else if(MotorSelect & 16){
		Set_Duty_Cycle(TIM5,3,Duty);
	}
	else if(MotorSelect & 32){
		Set_Duty_Cycle(TIM11,1,Duty);
	}
	else if(MotorSelect & 64){
		Set_Duty_Cycle(TIM5,4,Duty);
	}
	else if(MotorSelect & 128){
		Set_Duty_Cycle(TIM11,1,Duty);	//Motor 8
	}

	return 0;
}

void Stop_All_Motors(){

	Set_Duty_Cycle(TIM5,2,Stop_Duty);
	Set_Duty_Cycle(TIM1,4,Stop_Duty);
	Set_Duty_Cycle(TIM1,1,Stop_Duty);
	Set_Duty_Cycle(TIM5,1,Stop_Duty);
	Set_Duty_Cycle(TIM5,3,Stop_Duty);
	Set_Duty_Cycle(TIM5,4,Stop_Duty);
	Set_Duty_Cycle(TIM11,1,Stop_Duty);// since two of the outputs on the motorboard are supplied by the same timer/channel we only need 7 statements here
}


// End

//Motor Power outlet enables


void Disable_Motor_Power_Outputs(){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
}
void Enable_Motor_Power_Outputs(){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

// End


