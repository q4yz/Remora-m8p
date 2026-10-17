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
	// enable the DWT cycle counter, used to time the minimum step pulse width
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	this->postDelayCycles = (uint32_t)(((uint64_t)SystemCoreClock * STEP_PULSE_MIN_NS) / 1000000000ULL);

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
		// hold outputs (step pins) high for the minimum pulse width before running the post vector
		uint32_t start = DWT->CYCCNT;
		while ((DWT->CYCCNT - start) < this->postDelayCycles);

		for (iter = vThreadPost.begin(); iter != vThreadPost.end(); ++iter) (*iter)->runModulePost();
	}
}
