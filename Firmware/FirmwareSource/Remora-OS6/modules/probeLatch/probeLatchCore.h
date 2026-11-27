#ifndef PROBELATCHCORE_H
#define PROBELATCHCORE_H

// The probe latch's logic, without the hardware (PC tested: Firmware/Tests/probe).
//
// Run once per base thread tick (40 kHz: 25 us) with the probe's state and the Z
// step generator's step count of that tick. On the first trigger edge after the
// probe was released for armTicks it captures the step count; later edges while
// triggered, or chatter shorter than armTicks, do not overwrite it. Each capture
// counts on (0..3), so the host sees that a new one happened.
//
// Reported as one float (a Remora process variable): count * 4 + captures. A float
// holds every integer below 2^24 exactly, so counts within +-4194303 steps.

#include <cstdint>

class ProbeLatchCore
{
public:
	explicit ProbeLatchCore(uint32_t armTicks) : armTicks(armTicks) {}

	// one base thread tick; true when this tick captured
	bool tick(bool triggered, int32_t stepCount)
	{
		if (!triggered)
		{
			if (this->released < this->armTicks) ++this->released;
			if (this->released >= this->armTicks) this->armed = true;
			return false;
		}
		this->released = 0;
		if (!this->armed) return false;
		this->armed = false;
		this->count = stepCount;
		this->captures = (this->captures + 1) & 3;
		return true;
	}

	// the process variable: count * 4 + captures (exact while |count| < 2^22)
	float packed() const
	{
		return (float)((int64_t)this->count * 4 + this->captures);
	}

	bool isArmed() const { return this->armed; }
	int32_t latchedCount() const { return this->count; }
	uint8_t captureCount() const { return this->captures; }

private:
	uint32_t armTicks;
	uint32_t released = 0;
	bool armed = false;
	int32_t count = 0;
	uint8_t captures = 0;
};

#endif
