#include "pruThread.h"
#include "modules/module.h"
#include "configuration.h"


using namespace std;

// Thread constructor
pruThread::pruThread(TIM_TypeDef* timer, IRQn_Type irq, uint32_t frequency) :
	timer(timer),
	irq(irq),
	frequency(frequency),
	hasThreadPost(false),
	maxTicks(0),
	overruns(0),
	sumTicks(0),
	runs(0)
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

uint32_t pruThread::takeAverage(void)
{
	__disable_irq();
	uint64_t sum = this->sumTicks;
	uint32_t n = this->runs;
	this->sumTicks = 0;
	this->runs = 0;
	__enable_irq();
	return n ? (uint32_t)(sum / n) : 0;
}


void pruThread::run(void)
{
	uint32_t begin = DWT->CYCCNT;

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

	// the update interrupt flag was cleared on entry: set again means this run took a whole period or more
	uint32_t ticks = DWT->CYCCNT - begin;
	this->runs++;
	this->sumTicks += ticks;
	if (this->timer->SR & TIM_SR_UIF) this->overruns++;
	if (ticks > this->maxTicks) this->maxTicks = ticks;
}
