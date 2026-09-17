#ifndef NUCLEO_LED_UNIT_H
#define NUCLEO_LED_UNIT_H

#include "common.h"
#define LEDS_PORT   GPIOB

//port number
typedef enum{
    LED1 = 0, //Green
    LED2 = 7, //Blue
    LED3 = 14 //Red
} nucleo_led;


void LEDs_Init(void);
void toggle_led(nucleo_led led);
void set_led(nucleo_led led);
void reset_led(nucleo_led led);

#endif

