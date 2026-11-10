#include "commsMonitor.h"

#include "extern.h"

volatile bool commsLost = true;
volatile bool safeExpired = false;
uint32_t safeTimeout = SAFE_TIMEOUT_DEFAULT_S;


CommsMonitor::CommsMonitor(RemoraComms* comms) :
	comms(comms),
	lostPeriods(0)
{
}

void CommsMonitor::update()
{
	// register in the servo thread right after the comms module, which updates the status
	// an MCU over temperature shutdown puts every output in its safe state for good, like a lost link
	commsLost = !this->comms->getStatus() || mcuShutdown;

	// the safe timeout, counted in servo periods (no clock needed, every board family)
	if (!commsLost)
	{
		this->lostPeriods = 0;
		safeExpired = false;
		return;
	}
	if (safeExpired || !safeTimeout) return;
	if (++this->lostPeriods >= safeTimeout * servo_freq) safeExpired = true;
}
