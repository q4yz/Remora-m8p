#ifndef MCUTEMP_H
#define MCUTEMP_H

/***********************************************************************
    MCU over temperature shutdown (STM32G0: Manta M8P)

    Always on, it needs no config.txt entry: the STM32's internal
    temperature sensor is read 10 times a second in the servo thread
    (the same context as the thermistors, so the ADC is never used
    twice at once). At MCU_TEMP_SHUTDOWN the board shuts down for good:

    - every output goes to its safe state, as when LinuxCNC is gone
      (heaters and PWM to their safe duty, digital outputs to their safe
      level, the CAN nodes too), the step generators stop and disable
      their drivers
    - every SPI reply carries the PRU_OVERTEMP header: remora-spi drops
      SPI-status (E-stop) and sets remora.PRU-overtemp
    - it stays like that until the board is reset or power cycled

    A sensor that gives no plausible reading for 2 s counts as over
    temperature too (fail safe).

    The optional "MCU Temperature" module reports the temperature to
    LinuxCNC as a process variable, for a lower threshold of your own in
    HAL (the second line, this shutdown is the first):

        { "Type": "MCU Temperature", "Thread": "Servo", "Comment": "MCU", "PV[i]": 5 }
************************************************************************/

#include <cstdint>

#include "mbed.h"
#include "modules/module.h"

#include "extern.h"

#if defined(TARGET_STM32G0)
#define MCU_TEMP_SUPPORTED
#endif

#define MCU_TEMP_SHUTDOWN       90.0f   // degC die temperature: STM32G0B1xxT6 is rated for TJ 105 degC
#define MCU_TEMP_WARNING        80.0f   // degC, printed on the console
#define MCU_TEMP_TRIP_READINGS  3       // consecutive readings at or above the limit (0.3 s)
#define MCU_TEMP_FAULT_READINGS 20      // consecutive implausible readings (2 s)
#define MCU_TEMP_UPDATE_HZ      10

// set once by the over temperature shutdown, cleared only by a reset
extern volatile bool mcuShutdown;
// the last filtered die temperature (degC), for the console
extern volatile float mcuTemperature;

void createMcuTemperature(void);        // "MCU Temperature" module: report to a PV
void startMcuTempGuard(void);           // the always-on shutdown, call once in loadModules
void reportMcuTemp(void);               // main loop: console messages (not from the interrupt)

class McuTemp : public Module
{
    private:

#ifdef MCU_TEMP_SUPPORTED
        AnalogIn* sensor;
        AnalogIn* vref;
#endif
        volatile float* ptrFeedback;    // LinuxCNC process variable, nullptr without the module
        volatile int32_t* ptrTxHeader;
        float filtered;
        int   overCount;
        int   faultCount;
        uint8_t pending;                // update() calls until a postponed reading (0: none)

        float read(bool &valid);

    public:

        McuTemp(volatile int32_t &ptrTxHeader);

        void setFeedback(volatile float* feedback) { this->ptrFeedback = feedback; }

        virtual void update(void);
        virtual void slowUpdate(void);
};

#endif
