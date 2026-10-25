#include "interrupt.h"
#include "stm32g0xx_hal.h"

#include <cstdio>

// Define the vector table, it is only declared in the class declaration
Interrupt* Interrupt::ISRVectorTable[] = {0};

// Constructor
Interrupt::Interrupt(void){}


// Methods

void Interrupt::Register(int interruptNumber, Interrupt* intThisPtr)
{
	printf("Registering interrupt for interrupt number = %d\n", interruptNumber);
	ISRVectorTable[interruptNumber] = intThisPtr;
}

void Interrupt::TIM1_Wrapper(void)
{
	ISRVectorTable[TIM1_BRK_UP_TRG_COM_IRQn]->ISR_Handler();
}

void Interrupt::TIM2_Wrapper(void)
{
	ISRVectorTable[TIM2_IRQn]->ISR_Handler();
}

void Interrupt::TIM6_Wrapper(void)
{
	ISRVectorTable[TIM6_DAC_LPTIM1_IRQn]->ISR_Handler();
}

void Interrupt::TIM7_Wrapper(void)
{
	ISRVectorTable[TIM7_LPTIM2_IRQn]->ISR_Handler();
}

void Interrupt::TIM3_Wrapper(void)
{
	ISRVectorTable[TIM3_TIM4_IRQn]->ISR_Handler();
}