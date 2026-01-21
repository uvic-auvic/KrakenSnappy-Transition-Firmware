
#include <Initialize.h>
/*
 //////////////////// STM32 Initialization functions //////////////////////////
 *
 */


void AF_INIT( GPIO_TypeDef* GPIOx, unsigned int PIN, vu32 AF){

	if(GPIOx == GPIOA){

			RCC->AHB1ENR |= (vu16) 1;
		}
		else if(GPIOx == GPIOB){
			RCC->AHB1ENR |= (vu16) (1 << 1);
		}
		else if(GPIOx == GPIOC){
			RCC->AHB1ENR |= (vu16) (1 << 2);
		}
		else if(GPIOx == GPIOH){
			RCC->AHB1ENR |= (vu16) (1 << 7);

		}

		GPIOx->MODER |= (vu32) (2 << 2*PIN);
		GPIOx->OTYPER &= ~((vu16) (1 << PIN));
		GPIOx->OSPEEDR &= ~((vu32) (3 << 2*PIN));
		GPIOx->PUPDR   |= (vu32) (2 << 2*PIN);

		if(PIN < 8){
		GPIOx->AFR[0] |= ( AF << 4*PIN);
		}
		else{
		GPIOx->AFR[1] |= (AF << 4*(PIN-8));
		}


}

//Initilaize general timer

void INIT_TIM(TIM_TypeDef* TIMx, vu16 Auto_Reload_Register, vu16 Prescalar_Register){

vu16  Alignment = 0;
vu16 Count_Direction = 0;
vu16 Clock_Division = 0;



if(TIMx == TIM5){
Count_Direction = 0;
Alignment = 0;
};

if(TIMx == TIM1){
RCC->APB2ENR |= (vu32) 1;}
else if(TIMx == TIM5){
RCC->APB1ENR |= (vu32) (1 << 3);
}
else if(TIMx == TIM6){
RCC->APB1ENR |= (vu32) (1 << 4);

}
else if(TIMx == TIM9){
RCC->APB2ENR |= (vu32) (1 << 16);

}
else if(TIMx == TIM11){
RCC->APB2ENR |= (vu32) (1 << 18);
}
else{
return;
}



TIMx->CR1 |= (vu16)((Clock_Division << 8)|((vu16) 1 << 7)|(Alignment << 5)|(Count_Direction << 4)|( (vu16) 0 << 3));
TIMx->ARR = Auto_Reload_Register;
TIMx->PSC |= Prescalar_Register;


}


//Initializes a given timer
//TIM5 can only be in edge-aligned upcounting mode
void INIT_TIM_PWM(TIM_TypeDef* TIMx, vu16 Auto_Reload_Register, vu16 Prescalar_Register){
/* TO DO
 * Ask for:
 * TIMx_ARR value
 * Tim_CCRx value
 * edge/centre aligned mode
 * pwm mode
 *
 *
 * Set:
 *  ARPE bit in TIM timex_cr1
 *  enable ocpe in timx_ccmpx
 * Set ug bit in timx_EGR register *look into what this does
 * OCX polarity programable in ccxp bit in Timx_CCER
 * Ocx output enable through ccxe bit in TIMx_CCER register
 *
 */
//Ensuring TIM5 is in the correct mode
	vu16  Alignment = 0;
	vu16 Count_Direction = 0;
	vu16 Clock_Division = 0;



if(TIMx == TIM5){
	Count_Direction = 0;
	Alignment = 0;
};

if(TIMx == TIM1){
	RCC->APB2ENR |= (vu32) 1;}
else if(TIMx == TIM5){
	RCC->APB1ENR |= (vu32) (1 << 3);
}
else if(TIMx == TIM6){
	RCC->APB1ENR |= (vu32) (1 << 4);

}
else if(TIMx == TIM9){
	RCC->APB2ENR |= (vu32) (1 << 16);

}
else if(TIMx == TIM11){
	RCC->APB2ENR |= (vu32) (1 << 18);
}
else{
	return;
}



TIMx->CR1 |= (vu16)((Clock_Division << 8)|((vu16) 1 << 7)|(Alignment << 5)|(Count_Direction << 4)|( (vu16) 0 << 3));
TIMx->ARR = Auto_Reload_Register;
TIMx->PSC |= Prescalar_Register;
}

void Start_TIM(TIM_TypeDef* TIMx, TIM_MODE Mode){


	TIMx->EGR |= (vu16) 1;



	TIMx->CR1 |= (vu16) 1;

}

//Starts a PWM on a given timer/channel. Input desired duty cycle, and frequentcy. will return 1 if valid inputs
void Start_PWM(TIM_TypeDef* TIMx,unsigned int TIM_CH, float Duty_Cycle_Percent, GPIO_TypeDef* GPIOx, unsigned int PIN, vu32 AF){

	//caluculate needed compare register based on frequentcy, duty cycle and etc return some value to specify invalid input.
	vu16 Compare_Register = (Duty_Cycle_Percent*.01)*TIMx->ARR;

	if(TIMx == TIM1){
		TIMx->BDTR |= (vu16) (1 << 15);
	}

	if(TIM_CH ==1){
		TIMx->CCMR1 |= ((vu16) 6 << 4)|((vu16) 1 << 3);
		TIMx->CCR1 |= (vu16) Compare_Register;
		TIMx->CCER |= (vu16) 1;
	}

	else if(TIM_CH == 2){
		TIMx->CCMR1 |= ((vu16) 6 << 12)|((vu16) 1 << 11);
		TIMx->CCR2 |= Compare_Register;
		TIMx->CCER |= (vu16) 1 << 4;
	}
	else if(TIM_CH == 3){
		TIMx->CCMR2 |= ((vu16) 6 << 4)|((vu16) 1 << 3);
		TIMx->CCR3 |= Compare_Register;
		TIMx->CCER |= (vu16) (1 << 8);
	}
	else if (TIM_CH == 4){
		TIMx->CCMR2 |= ((vu16) 6 << 12)|((vu16) 1 << 11);
		TIMx->CCR4 |= Compare_Register;
		TIMx->CCER |= (vu16) (1 << 12);
	}

	AF_INIT(GPIOx,PIN,AF);

	TIMx->EGR |= (vu16) 1;
	TIMx->CR1 |= (vu16) 1;
	}

void Set_Duty_Cycle(TIM_TypeDef* TIMx,unsigned int TIM_CH, float Duty_Cycle_Percent){


	vu16 Compare_Register = (Duty_Cycle_Percent*.01)*TIMx->ARR;

	if(TIM_CH ==1){
		TIMx->CCR1 = Compare_Register;
		}
		else if(TIM_CH == 2){
		TIMx->CCR2 = Compare_Register;
		}
		else if(TIM_CH == 3){
		TIMx->CCR3 = Compare_Register;
		}
		else if (TIM_CH == 4){
		TIMx->CCR4 = Compare_Register;
		}


}

/*
 //////////////////// End //////////////////////////
 *
 */
