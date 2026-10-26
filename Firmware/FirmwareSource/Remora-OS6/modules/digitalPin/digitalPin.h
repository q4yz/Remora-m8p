#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include <cstdint>

#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "extern.h"

void createDigitalPin(void);

class DigitalPin : public Module
{
	private:

		typedef volatile uint16_t __attribute__((aligned(1))) unaligned_u16;
		unaligned_u16 *ptrData; 		// pointer to the data source, rxData.outputs is not 16-bit aligned
		int bitNumber;				// location in the data source
		bool invert;
		bool safeState;				// output level while LinuxCNC is disconnected
		int mask;

		int mode;
        int modifier;
		std::string portAndPin;

		Pin *pin;

	public:

        DigitalPin(volatile uint16_t&, int, std::string, int, bool, int);
		void setSafeState(bool);
		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
