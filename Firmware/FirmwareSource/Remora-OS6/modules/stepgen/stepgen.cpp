#include "stepgen.h"


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

void createStepgen()
{
    const char* comment = module["Comment"];
    printf("%s\n",comment);

    int joint = module["Joint Number"];
    const char* enable = module["Enable Pin"];
    const char* step = module["Step Pin"];
    const char* dir = module["Direction Pin"];

    // configure pointers to data source and feedback location
    ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    // create the step generator, register it in the thread
    Module* stepgen = new Stepgen(base_freq, joint, enable, step, dir, STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(stepgen);
    baseThread->registerModulePost(stepgen);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

Stepgen::Stepgen(int32_t threadFreq, int jointNumber, std::string enable, std::string step, std::string direction, int stepBit, volatile int32_t &ptrFrequencyCommand, volatile int32_t &ptrFeedback, volatile uint8_t &ptrJointEnable) :
	jointNumber(jointNumber),
	enable(enable),
	step(step),
	direction(direction),
	stepBit(stepBit),
	ptrFrequencyCommand(&ptrFrequencyCommand),
	ptrFeedback(&ptrFeedback),
	ptrJointEnable(&ptrJointEnable)
{
	this->enablePin = new Pin(this->enable, OUTPUT);			// create Pins
	this->stepPin = new Pin(this->step, OUTPUT);
	this->directionPin = new Pin(this->direction, OUTPUT);
    this->rawCount = 0;
	this->DDSaccumulator = 0;
	this->frequencyCommand = 0;
	this->DDSaddValue = 0;
	// DDS add value per Hz in 32.32 fixed point, keeps floating point maths out of the base thread
	// (the Cortex-M0+ of the STM32G0 has no FPU)
	this->frequencyScale = ((int64_t)1 << (this->stepBit + 32)) / threadFreq;
	this->mask = 1 << this->jointNumber;
	this->isEnabled = false;
	this->isForward = false;
}


void Stepgen::update()
{
	// Use the standard Module interface to run makePulses()
	this->makePulses();
}

void Stepgen::updatePost()
{
	this->stopPulses();
}

void Stepgen::slowUpdate()
{
	return;
}

void Stepgen::makePulses()
{
	int32_t stepNow = 0;

	this->isEnabled = ((*(this->ptrJointEnable) & this->mask) != 0);

	if (this->isEnabled == true)  												// this Step generator is enables so make the pulses
	{
		this->enablePin->set(false);                                			// Enable the driver - CHANGE THIS TO MAKE THE OUTPUT VALUE CONFIGURABLE???

		int32_t command = *(this->ptrFrequencyCommand);            				// Get the latest frequency command via pointer to the data source
		if (command != this->frequencyCommand)									// it only changes once per servo period, rescale only then
		{
			this->frequencyCommand = command;
			int64_t addValue = (int64_t)command * this->frequencyScale;			// Scale the frequency command to get the DDS add value,
			this->DDSaddValue = (int32_t)(addValue >= 0 ? addValue >> 32 : -((-addValue) >> 32));	// truncated towards zero
		}
		stepNow = this->DDSaccumulator;                           				// Save the current DDS accumulator value
		this->DDSaccumulator += this->DDSaddValue;           	  				// Update the DDS accumulator with the new add value
		stepNow ^= this->DDSaccumulator;                          				// Test for changes in the low half of the DDS accumulator
		stepNow &= (1L << this->stepBit);                         				// Check for the step bit
		//this->rawCount = this->DDSaccumulator >> this->stepBit;   				// Update the position raw count

		if (this->DDSaddValue > 0)												// The sign of the DDS add value indicates the desired direction
		{
			this->isForward = true;
		}
		else //if (this->DDSaddValue < 0)
		{
			this->isForward = false;
		}

		if (stepNow)
		{
			this->directionPin->set(this->isForward);             		    // Set direction pin
			this->stepPin->set(true);										// Raise step pin - A4988 / DRV8825 stepper drivers only need 200ns setup time
            if (this->isForward)
            {
                ++this->rawCount;
            }
            else
            {
                --this->rawCount;
            }
			//*(this->ptrFeedback) = this->DDSaccumulator;                     // Update position feedback via pointer to the data receiver
            *(this->ptrFeedback) = this->rawCount;
            this->isStepping = true;
		}


	}
	else
	{
		this->enablePin->set(true);
	}

}

void Stepgen::stopPulses()
{
	this->stepPin->set(false);	// Reset step pin
	this->isStepping = false;
}


void Stepgen::setEnabled(bool state)
{
	this->isEnabled = state;
}
