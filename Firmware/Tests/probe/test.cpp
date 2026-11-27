// PC test of the probe latch (modules/probeLatch/probeLatchCore.h).
#include <cmath>
#include <cstdio>

#include "probeLatchCore.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

// decode like the backend does: v = count * 4 + captures
static void decode(float pv, long &count, int &captures)
{
	long v = std::lround(pv);
	captures = (int)(v & 3);
	count = (v - captures) / 4;
}

int main()
{
	const unsigned ARM = 80;  // 2 ms at 40 kHz
	ProbeLatchCore latch(ARM);
	long count; int captures;

	// not armed at power on: a probe that starts triggered captures nothing
	CHECK(!latch.tick(true, 100));
	CHECK(!latch.isArmed());

	// released for the arm time: armed
	for (unsigned i = 0; i < ARM; ++i) latch.tick(false, 100);
	CHECK(latch.isArmed());

	// a probe move: the count runs down, the first trigger edge captures that tick's count
	int32_t z = 5000;
	bool captured = false;
	for (int t = 0; t < 1000 && !captured; ++t)
	{
		z -= 1;                               // one step per tick
		captured = latch.tick(z <= 4321, z);
	}
	CHECK(captured);
	decode(latch.packed(), count, captures);
	CHECK(count == 4321 && captures == 1);

	// chatter at the switching point (shorter than the arm time) and the move running on
	// after the trigger: the first capture stays
	for (int t = 0; t < 30; ++t) latch.tick(t % 2 == 0, 4300 - t);
	for (int t = 0; t < 200; ++t) latch.tick(true, 4250 - t);
	decode(latch.packed(), count, captures);
	CHECK(count == 4321 && captures == 1);

	// retract and release long enough: the next probe move captures again
	for (unsigned i = 0; i < ARM; ++i) latch.tick(false, 4800);
	CHECK(latch.tick(true, 4322));
	decode(latch.packed(), count, captures);
	CHECK(count == 4322 && captures == 2);

	// negative counts (below where the board started) and the capture counter wrapping 3 -> 0
	for (int k = 0; k < 2; ++k)
	{
		for (unsigned i = 0; i < ARM; ++i) latch.tick(false, 0);
		latch.tick(true, -123457 - k);
	}
	decode(latch.packed(), count, captures);
	CHECK(count == -123458 && captures == 0);

	// exact up to the documented range (|count| < 2^22)
	for (unsigned i = 0; i < ARM; ++i) latch.tick(false, 0);
	latch.tick(true, 4194303);
	decode(latch.packed(), count, captures);
	CHECK(count == 4194303 && captures == 1);
	for (unsigned i = 0; i < ARM; ++i) latch.tick(false, 0);
	latch.tick(true, -4194303);
	decode(latch.packed(), count, captures);
	CHECK(count == -4194303 && captures == 2);

	std::printf("probe latch: %s\n", failures ? "FAILED" : "PASSED");
	return failures ? 1 : 0;
}
