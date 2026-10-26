#include "mbed.h"
#include "stm32g0xx_hal.h"

#include <stdio.h>

#include "interrupt.h"
#include "timerInterrupt.h"
#include "timer.h"
#include "pruThread.h"



// Timer constructor
pruTimer::pruTimer(TIM_TypeDef* timer, IRQn_Type irq, uint32_t frequency, pruThread* ownerPtr):
	timer(timer),
	irq(irq),
	frequency(frequency),
	timerOwnerPtr(ownerPtr)
{
	interruptPtr = new TimerInterrupt(this->irq, this);	// Instantiate a new Timer Interrupt object and pass "this" pointer

	this->startTimer();
}


void pruTimer::timerTick(void)
{
	//Do something here
	this->timerOwnerPtr->run();
}



void pruTimer::startTimer(void)
{
    uint32_t TIM_CLK;

    if (this->timer == TIM1)
    {
        printf("	power on Timer 1\n\r");
        __HAL_RCC_TIM1_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else if (this->timer == TIM2)
    {
        printf("	power on Timer 2\n\r");
        __HAL_RCC_TIM2_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else if (this->timer == TIM3)
    {
        printf("	power on Timer 3\n\r");
        __HAL_RCC_TIM3_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else if (this->timer == TIM6)
    {
        printf("	power on Timer 6\n\r");
        __HAL_RCC_TIM6_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else if (this->timer == TIM7)
    {
        printf("	power on Timer 7\n\r");
        __HAL_RCC_TIM7_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else
    {
        printf("	Error - unsupported thread timer\n\r");
        return;
    }

    if (TIM_CLK / TIM_PSC / this->frequency > 0x10000 && this->timer != TIM2)
    {
        // TIM3, TIM6 and TIM7 are 16 bit, at 64 MHz the lowest thread frequency is 977 Hz
        printf("	Error - thread frequency %lu Hz too low for a 16 bit timer\n\r", (unsigned long)this->frequency);
        return;
    }


    //timer uptade frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1)

    this->timer->CR2 &= 0;                                            // UG used as trigg output
    this->timer->PSC = TIM_PSC-1;                                     // prescaler
    this->timer->ARR = ((TIM_CLK / TIM_PSC / this->frequency) - 1);   // period           
    this->timer->EGR = TIM_EGR_UG;                                    // reinit the counter
    this->timer->DIER = TIM_DIER_UIE;                                 // enable update interrupts

    this->timer->CR1 |= TIM_CR1_CEN;                                  // enable timer

    NVIC_EnableIRQ(this->irq);
}

void pruTimer::stopTimer()
{
    NVIC_DisableIRQ(this->irq);

    printf("	timer stop\n\r");
    this->timer->CR1 &= (~(TIM_CR1_CEN));     // disable timer
}