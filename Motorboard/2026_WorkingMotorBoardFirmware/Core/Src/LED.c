#include "LED.h"


//There are 4 leds input numbers 1-4 depending on the LED you want to write

// Inputs:
		//pin: led # between 0-3
		//state: input desired state 1 or greater LED on, 0 LED 0ff
//Outputs:
		//returns -1 if bad pin/LED select
		//returns -2 if bad state ie negative number

int UserWriteLED(int pin, int state){

	if(pin > 3 || pin < 0){
		return -1;
	}

	if(state > 0){
		HAL_GPIO_WritePin(GPIOC, pin, 1);
	}
	else if(state == 0){
		HAL_GPIO_WritePin(GPIOC, pin, 0);
	}

	return -2;
}
//There are 4 leds input numbers 1-4 depending on led to toggle
int UserToggleLED(int pin){

	if(pin > 3 || pin < 0){
			return -1;
		}

	HAL_GPIO_TogglePin(GPIOC, pin);
}



void LEDTimerToggle(uint16_t pin,int Timer, float LEDState1, float LEDState2){


	if(Timer == 0){
		HAL_GPIO_WritePin(GPIOC, pin, 0);
	}
	else if(Timer == 1){

		HAL_GPIO_WritePin(GPIOC, pin, LEDState1);
	}
	else if(Timer == 2){
		HAL_GPIO_WritePin(GPIOC, pin, LEDState2);
	}
	else if(Timer == 3){
		HAL_GPIO_WritePin(GPIOC, pin, 1);
	}


}

