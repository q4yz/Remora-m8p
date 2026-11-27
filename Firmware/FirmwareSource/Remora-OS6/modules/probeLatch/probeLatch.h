#ifndef PROBELATCH_H
#define PROBELATCH_H

#include <cstdint>
#include <string>

#include "modules/module.h"
#include "drivers/pin/pin.h"
#include "probeLatchCore.h"

#include "extern.h"

// "Probe Latch": the probe pin read in the base thread, the Z step generator's step count
// captured at its trigger edge (probeLatchCore.h), reported in a process variable. LinuxCNC
// only sees the probe once per servo period (1 ms): at 5 mm/s that is 0.005 mm, plus the
// jitter between the board's servo thread and the SPI frame. The latched count is exact to a
// base thread tick (25 us). config.txt:
//   {"Thread": "Base", "Type": "Probe Latch", "Comment": "probe", "Pin": "PB7",
//    "Joint Number": 2, "PV[i]": 3, "Modifier": "Pull Up", "Invert": "False"}
// The probe also needs its usual "Digital Pin" input (LinuxCNC's G38 stops on that one).

void createProbeLatch();

class ProbeLatch : public Module
{
private:
	Pin* pin;
	bool invert;
	volatile int32_t* ptrStepCount;
	volatile float* ptrPV;
	ProbeLatchCore core;

public:
	ProbeLatch(std::string portAndPin, int modifier, bool invert, volatile int32_t &ptrStepCount,
	           volatile float &ptrPV, uint32_t armTicks);

	virtual void update(void);
};

#endif
