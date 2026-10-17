#ifndef COMMSMONITOR_H
#define COMMSMONITOR_H

#include "modules/module.h"
#include "RemoraComms.h"

// true while LinuxCNC is not exchanging data with the board (also from power up until it first
// connects). Output modules then drive their configured safe state instead of the LinuxCNC command,
// so a lost connection can't leave a heater or spindle switched on.
extern volatile bool commsLost;

class CommsMonitor : public Module
{
	private:

		RemoraComms* comms;

	public:

		CommsMonitor(RemoraComms*);
		virtual void update(void);
};

#endif
