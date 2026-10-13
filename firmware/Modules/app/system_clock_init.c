#include "system_clock_init.h"

void SysTickInit(void){
	//	SysTick_CTRL_CLKSOURCE_Msk is AHB(108 Mhz)
	//SysTick->LOAD = (108000UL << SysTick_LOAD_RELOAD_Pos); //Systick Period = 1/108 Mhz * 108000 = 1 ms
	//Enable interrupts + enable count down from reload number(from load register)
	//SysTick->CTRL =  SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; 
	SysTick_Config(10000);//~90 us period for 108 MHz
}

void SysClockInit(void){
	//activate HSE as clock source [4 - 26 Mhz] but def HSE_VALUE = 25 Mhz in system_stm32f67xx.c,
//	RCC->CR |= RCC_CR_HSEON; 
//	while(!(RCC->CR & RCC_CR_HSERDY)); //wait for HSE initialization
	
	RCC->CR |= RCC_CR_HSION; //activate hsi source
	while(!(RCC->CR & RCC_CR_HSIRDY)); //wait for HSI initialization
	
	RCC->CR &= ~RCC_CR_PLLON; //reset PLL source during configuration - is it important?
	while(RCC->CR & RCC_CR_PLLRDY); //wait for PLL to reset
	
	/*
	-- PLL config -- 
	16 times divition of source clck(HSI) -> get PLLM 1MHz 
	216 multiplication of PLLM -> get PLLN 216 Mhz
  set div by 2 (0b00) -> nothing to set -> get final 108 Mhz 
	*/
	RCC-> PLLCFGR = (16UL << RCC_PLLCFGR_PLLM_Pos) | (216UL << RCC_PLLCFGR_PLLN_Pos);
//	RCC-> PLLCFGR = (25UL << RCC_PLLCFGR_PLLM_Pos) | (432UL << RCC_PLLCFGR_PLLN_Pos) | (RCC_PLLCFGR_PLLSRC_HSE);
	
	
	//enable PLL as sys clck source 
	RCC -> CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	//flash latency (3 wait states ~ 4 cpu cycles) 
	do{
		FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
	}while((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS);

	/**
	set PLL as sys clock source -> APB/AHB
	APB1 bus max clock freq is 54 MHz -> div4: 216/4
	APB2 bus max clock fre is 108 MHz -> div2: 216/2
	**/
	RCC->CFGR = RCC_CFGR_SW_PLL | RCC_CFGR_PPRE1_DIV2; 

}



