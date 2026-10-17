#include "commsMonitor.h"

volatile bool commsLost = true;


CommsMonitor::CommsMonitor(RemoraComms* comms) :
	comms(comms)
{
}

void CommsMonitor::update()
{
	// register in the servo thread right after the comms module, which updates the status
	commsLost = !this->comms->getStatus();
}
