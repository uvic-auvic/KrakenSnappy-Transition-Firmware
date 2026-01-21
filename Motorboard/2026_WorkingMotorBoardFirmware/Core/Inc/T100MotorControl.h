/*
 * Timers.h
 *
 *  Created on: Mar 6, 2023
 *      Author: Matthew Atagi
 */
#ifndef T100MotorControl_H
#define T100MotorControl_H

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

void Initialize_Motors();


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
int Bitwise_Motor_Control(vu8 MotorSelect,float Speed);


void Stop_All_Motors();


void Disable_Motor_Power_Outputs();


void Enable_Motor_Power_Outputs();


#endif /* SRC_TIMERS_H_ */
