#ifndef SDBUS_H
#define SDBUS_H

/***********************************************************************
    The board's SD card bus, driven only while the card is in use

    For a board whose SD card shares its lines with something else (the
    Manta M8P: with the stepper driver sockets): its board.cpp has the
    two functions. Every other board gets the empty defaults (sdBus.cpp).
************************************************************************/

void sdBusAcquire(void);        // before the card is accessed
void sdBusRelease(void);        // after it: the lines are let go

#endif
