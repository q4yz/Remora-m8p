#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "stm32g0xx_hal.h"
#include "timer.h"

// Standard Template Library (STL) includes
#include <algorithm>
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
        uint32_t postDelayTicks;	// minimum timer ticks between main and post vectors (step pulse width)

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

		// load measurement: the longest run() in timer ticks (SystemCoreClock / TIM_PSC) and the
		// runs that took longer than a thread period (the next timer update was already pending).
		// Mirrored into the TAMP backup registers, which survive a watchdog reset: the next boot
		// of a debug build prints what the threads did before it (see bkpLoad()).
		volatile uint32_t	maxTicks;
		volatile uint32_t	overruns;
		volatile uint64_t	sumTicks;			// all runs since takeAverage(), for the average load
		volatile uint32_t	runs;
		uint32_t			takeAverage(void);	// average ticks per run since the last call (main loop)
		uint32_t			periodTicks(void) const { return this->timer->ARR + 1; }

		static void bkpEnable(void);
		// slot 0 base thread (TIM6), 1 servo thread (TIM7): max ticks in BKP[2*slot], overruns in BKP[2*slot+1]
		static void bkpLoad(int slot, uint32_t* maxTicks, uint32_t* overruns);
		static void bkpClear(void);

	private:
		int bkpSlot;
};

#endif
