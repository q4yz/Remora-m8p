#include "hardwarePwm.h"
#include "extern.h"

using namespace std;

#define PWMPERIOD 200

HardwarePWM::HardwarePWM(volatile float &ptrPwmPulseWidth, int pwmPeriod, std::string pin) :
    ptrPwmPulseWidth(&ptrPwmPulseWidth),
    pwmPeriod(pwmPeriod),
	pin(pin)
{
    printf("Creating Hardware PWM at pin %s\n", this->pin.c_str());

    variablePeriod = false;
    safeDuty = 0;

    if (pwmPeriod == 0)
    {
        this->pwmPeriod = PWMPERIOD;
    }

    Pin* dummyPin = new Pin(pin, 1);
    this->pwmPin = dummyPin->hardware_pwm();

    if (this->pwmPin == NULL) {
        printf("  Error: Hardware PWM cannot this pin (Refer to Hardware Documents for Pin Information)\n");
        delete dummyPin;
        return;
    }

    this->pwmPin->period_us(this->pwmPeriod);
}


HardwarePWM::HardwarePWM(volatile float &ptrPwmPeriod, volatile float &ptrPwmPulseWidth, int pwmPeriod, std::string pin) :
    ptrPwmPeriod(&ptrPwmPeriod),
    ptrPwmPulseWidth(&ptrPwmPulseWidth),
    pwmPeriod(pwmPeriod),
	pin(pin)
{
    printf("Creating variable frequency Hardware PWM at pin %s\n", this->pin.c_str());

    variablePeriod = true;
    safeDuty = 0;

    if (pwmPeriod == 0)
    {
        this->pwmPeriod = PWMPERIOD;
    }

    Pin* dummyPin = new Pin(pin, 1);
    this->pwmPin = dummyPin->hardware_pwm();

    if (this->pwmPin == NULL) {
        printf("  Error: Hardware PWM cannot this pin (Refer to Hardware Documents for Pin Information)\n");
        delete dummyPin;
        return;
    }

    this->pwmPin->period_us(this->pwmPeriod);
}


void HardwarePWM::update()
{
    if (variablePeriod)
    {
        if (*(this->ptrPwmPeriod) != 0 && (*(this->ptrPwmPeriod) != this->pwmPeriod))
        {
            // PWM period has changed
            this->pwmPeriod = *(this->ptrPwmPeriod);
            this->pwmPin->period_us(this->pwmPeriod);
            this->pwmPulseWidth_us = (this->pwmPeriod * this->pwmPulseWidth) / 100.0;
            this->pwmPin->pulsewidth_us(this->pwmPulseWidth_us);
        }
    }

    // use the safe duty while LinuxCNC is disconnected
    float pulseWidth = commsLost ? this->safeDuty : *(this->ptrPwmPulseWidth);

    if (pulseWidth != this->pwmPulseWidth)
    {
        // PWM duty has changed
        this->pwmPulseWidth = pulseWidth;
        this->pwmPulseWidth_us = (this->pwmPeriod * this->pwmPulseWidth) / 100.0;
        this->pwmPin->pulsewidth_us(this->pwmPulseWidth_us);
    } 

    return;
}


void HardwarePWM::setSafeDuty(float duty)
{
    this->safeDuty = duty;
}


void HardwarePWM::slowUpdate()
{
	return;
}
