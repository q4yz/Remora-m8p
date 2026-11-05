#include "commsMonitor.h"

volatile bool commsLost = true;


CommsMonitor::CommsMonitor(RemoraComms* comms) :
	comms(comms)
{
}

void CommsMonitor::update()
{
	// register in the servo thread right after the comms module, which updates the status
	// an MCU over temperature shutdown puts every output in its safe state for good, like a lost link
	commsLost = !this->comms->getStatus() || mcuShutdown;
}
