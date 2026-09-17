#ifndef COMMON_H
#define COMMON_H

#include <stm32f767xx.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "system_clock_init.h" 
#include "led_unit.h"

#define SYS_TICK_PERIOD_US     90

void delay_ms(uint16_t milis);
//extern volatile uint32_t msCounter;
extern volatile uint32_t usCounter;
#endif

