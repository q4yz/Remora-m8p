#include "pruThread.h"
#include "modules/module.h"
#include "configuration.h"


using namespace std;

// Thread constructor
pruThread::pruThread(TIM_TypeDef* timer, IRQn_Type irq, uint32_t frequency) :
	timer(timer),
	irq(irq),
	frequency(frequency),
	hasThreadPost(false)
{
	// the Cortex-M0+ has no cycle counter, the minimum step pulse width is timed with the thread's
	// own timer counter, which counts at SystemCoreClock / TIM_PSC
	this->postDelayTicks = (uint32_t)(((uint64_t)SystemCoreClock / TIM_PSC * STEP_PULSE_MIN_NS) / 1000000000ULL);

	printf("Creating thread %d\n", this->frequency);
}

void pruThread::startThread(void)
{
	TimerPtr = new pruTimer(this->timer, this->irq, this->frequency, this);
}

void pruThread::stopThread(void)
{
    this->TimerPtr->stopTimer();
}


void pruThread::registerModule(Module* module)
{
	this->vThread.push_back(module);
}

void pruThread::registerModulePost(Module* module)
{
	this->vThreadPost.push_back(module);
	this->hasThreadPost = true;
}

void pruThread::unregisterModule(Module* module)
{
	iter = std::remove(vThread.begin(),vThread.end(), module);
    vThread.erase(iter, vThread.end());
}

void pruThread::run(void)
{
	// iterate over the Thread pointer vector to run all instances of Module::runModule()
	for (iter = vThread.begin(); iter != vThread.end(); ++iter) (*iter)->runModule();

    // iterate over the second vector that contains module pointers to run after (post) the main vector
	if (hasThreadPost)
	{
		// hold outputs (step pins) high for the minimum pulse width before running the post vector.
		// The counter wraps at ARR once per thread period, the wait is always shorter than that
		uint32_t period = this->timer->ARR + 1;
		uint32_t delay = this->postDelayTicks < period / 2 ? this->postDelayTicks : period / 2;
		uint32_t start = this->timer->CNT;
		uint32_t elapsed = 0;
		while (elapsed < delay)
		{
			uint32_t now = this->timer->CNT;
			elapsed = now >= start ? now - start : now + period - start;
		}

		for (iter = vThreadPost.begin(); iter != vThreadPost.end(); ++iter) (*iter)->runModulePost();
	}
}
