#ifndef COMMSMONITOR_H
#define COMMSMONITOR_H

#include "modules/module.h"
#include "RemoraComms.h"

// true while LinuxCNC is not exchanging data with the board (also from power up until it first
// connects). Output modules then drive their configured safe state instead of the LinuxCNC command,
// so a lost connection can't leave a heater or spindle switched on.
extern volatile bool commsLost;
extern volatile bool mcuShutdown;

// after safeTimeout seconds of commsLost every output whose safe value is on (a hotend fan at
// 100 %, a digital output safe high) goes off, as LinuxCNC's 0, until LinuxCNC is back: a fan
// must not run for ever with LinuxCNC in E-stop or gone. "Safe Timeout" in config.txt, 0: never.
#define SAFE_TIMEOUT_DEFAULT_S  240
extern volatile bool safeExpired;
extern uint32_t safeTimeout;

class CommsMonitor : public Module
{
	private:

		RemoraComms* comms;
		uint32_t     lostPeriods;           // servo periods with commsLost

	public:

		CommsMonitor(RemoraComms*);
		virtual void update(void);
};

#endif
