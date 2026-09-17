#include "led_unit.h"

void LEDs_Init(void){
    //Enable clock on GPIOB from AHB1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    //set gpiob0(led1 green) in output mode (ob01)
    GPIOB->MODER |= 1U << GPIO_MODER_MODER0_Pos; 
    
    //set gpiob0(led2 blue) in output mode (ob01)
    GPIOB->MODER |= 1U << GPIO_MODER_MODER7_Pos; 
    
    //set gpiob0(led3 red) in output mode (ob01)
    GPIOB->MODER |= 1U << GPIO_MODER_MODER14_Pos;   
}

void toggle_led(nucleo_led led){
    GPIOB->ODR ^= (1U << led);
}

void set_led(nucleo_led led){
    GPIOB->BSRR |= (1U << led);
}

void reset_led(nucleo_led led){
//    GPIOB->BSRR |= (1U << led);
    GPIOB->BSRR |= (1U << (led + 16));
}