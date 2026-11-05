#include "pruThread.h"
#include "modules/module.h"
#include "configuration.h"


using namespace std;


void pruThread::bkpEnable(void)
{
	// the TAMP backup registers need the RTC APB clock and backup domain write access
	RCC->APBENR1 |= RCC_APBENR1_PWREN | RCC_APBENR1_RTCAPBEN;
	(void)RCC->APBENR1;
	PWR->CR1 |= PWR_CR1_DBP;
}

void pruThread::bkpLoad(int slot, uint32_t* maxTicks, uint32_t* overruns)
{
	volatile uint32_t* bkp = &TAMP->BKP0R;
	*maxTicks = bkp[2 * slot];
	*overruns = bkp[2 * slot + 1];
}

void pruThread::bkpClear(void)
{
	volatile uint32_t* bkp = &TAMP->BKP0R;
	for (int i = 0; i < 4; i++) bkp[i] = 0;
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
	this->bkpSlot = (timer == TIM6) ? 0 : (timer == TIM7) ? 1 : -1;
	if (this->bkpSlot >= 0) bkpEnable();

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

	// the update interrupt fires at CNT 0 and its flag was cleared on entry: CNT is the time
	// this run took (interrupt latency included), a set flag means it took a whole period or more
	uint32_t ticks = this->timer->CNT;
	this->runs++;
	if (this->timer->SR & TIM_SR_UIF)
	{
		this->sumTicks += ticks + this->timer->ARR + 1;
		this->overruns++;
		if (this->bkpSlot >= 0) (&TAMP->BKP0R)[2 * this->bkpSlot + 1] = this->overruns;
		return;
	}
	this->sumTicks += ticks;
	if (ticks > this->maxTicks)
	{
		this->maxTicks = ticks;
		if (this->bkpSlot >= 0) (&TAMP->BKP0R)[2 * this->bkpSlot] = ticks;
	}
}
