#ifndef LED_H_
#define LED_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

int UserWriteLED(int pin, int state);
int UserToggleLED(int pin);
void LEDTimerToggle(uint16_t pin,int Timer, float LEDState1, float LEDState2);

#endif
