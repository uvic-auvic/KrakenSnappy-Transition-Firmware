



#ifndef Initialize_H_
#define Initialize_H_

#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <stdint.h>

#define vu8 volatile uint8_t
#define vu16 volatile uint16_t
#define vu32 volatile uint32_t

#define v8 volatile int8_t
#define v16 volatile int16_t
#define v32 volatile int32_t


//Parameters for T100 Thruster Control

#define PrescalerRegister 84 - 1 //Divide timer clock (84MHZ) by x + 1
#define INITIALIZE_DUTY 7.5 //7.5 for 1500 us
#define Stop_Duty 7.5
#define PWM_PERIOD 20000    //Period of PWM timers
#define MOTOR_RANGE 400		//Motor power input range (400 Max) leave at 200 unless told otherwise by Elec Lead


//End//

typedef enum  {

	Counter = 0,
	PWM,
	One_Pulse_Mode

} TIM_MODE;

void INIT_TIM_INIT(TIM_TypeDef* TIMx, vu16 Auto_Reload_Preload_Enable, vu16  Alignment, vu16 One_Pulse_Mode, vu16 Count_Direction,  vu16 Update_Request_Source, vu16 Prescaler, vu16 Clock_Division, vu16 Update_Disable);

void INIT_TIM_PWM(TIM_TypeDef* TIMx, vu16 Auto_Reload_Register, vu16 Prescalar_Register);

void Start_PWM(TIM_TypeDef* TIMx,unsigned int TIM_CH, float Duty_Cycle_Percent, GPIO_TypeDef* GPIOx, unsigned int PIN, vu32 AF);

void Start_TIM(TIM_TypeDef* TIMx, TIM_MODE Mode);

void Stop_TIM(TIM_TypeDef* TIMx);

void Set_Duty_Cycle(TIM_TypeDef* TIMx,unsigned int TIM_CH, float Duty_Cycle_Percent);

#endif
