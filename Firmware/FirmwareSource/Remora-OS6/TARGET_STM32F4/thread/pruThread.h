#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "stm32f4xx_hal.h"
#include "timer.h"

// Standard Template Library (STL) includes
#include <iostream>
#include <vector>

using namespace std;

class Module;

class pruThread
{

	private:

		pruTimer* 		    TimerPtr;
	
		TIM_TypeDef* 	    timer;
		IRQn_Type 			irq;
		uint32_t 			frequency;

        bool hasThreadPost;		// run updatePost() vector
        uint32_t postDelayCycles;	// minimum CPU cycles between main and post vectors (step pulse width)

		vector<Module*> vThread;		// vector containing pointers to Thread modules
        vector<Module*> vThreadPost;		// vector containing pointers to Thread modules that run after the main vector modules
		vector<Module*>::iterator iter;

	public:

		pruThread(TIM_TypeDef* timer, IRQn_Type irq, uint32_t frequency);

		void registerModule(Module *module);
        void registerModulePost(Module *module);
        void unregisterModule(Module *module);
		void startThread(void);
        void stopThread(void);
		void run(void);

		// load measurement in CPU cycles (DWT): the longest run(), the runs that took longer than a
		// thread period (the next timer update was already pending), the average (debug console)
		volatile uint32_t	maxTicks;
		volatile uint32_t	overruns;
		volatile uint64_t	sumTicks;			// all runs since takeAverage(), for the average load
		volatile uint32_t	runs;
		uint32_t			takeAverage(void);	// average cycles per run since the last call (main loop)
		uint32_t			periodTicks(void) const { return SystemCoreClock / this->frequency; }
};

#endif

