#ifndef SOFTWARESERIAL_H
#define SOFTWARESERIAL_H

//#include "LPC17xx.h"
#include "mbed.h"
#include <cstdint>
#include <string>
#include "configuration.h"
#include "pin.h"


#define FORCE_BAUD_RATE 19600 //9600
#define IN_BUF_SIZE     64
#define TX_BITS         10     // 1 Startbit, 8 Databits, 1 Stopbit = 10 Bits/Frame
#define RX_BITS         8      // startbit and stopbit parsed internally (see ISR)
//#define OVERSAMPLE      3


class SoftwareSerial
{
    private:

    //static SoftwareSerial* instance;        // there can only be one
    //void (*_rit_isr)(void);                 // storage for an appended isr function
    //void ritisr(void);                      // the default (instance) isr
    //static void _ritisr(void);              // the actual static isr

    std::string     TXportAndPin;
    std::string     RXportAndPin;
    Pin*            txpin;
    Pin*            rxpin;

    //Ticker          ticker;
    
    // shared with the tick interrupt: volatile, or the main loop's wait for activeTx may be hoisted
    volatile unsigned char inbuf[IN_BUF_SIZE];
    volatile unsigned char qin;
    unsigned char   qout;

    int32_t  baudRate;

    volatile bool    activeTx;
    volatile bool    activeRx;
    bool             halfDuplex;
    volatile bool    outputPending;

    volatile int32_t rxTickCnt;
    volatile int32_t txTickCnt;
    volatile int32_t txBitCnt;
    volatile int32_t rxBitCnt;
    volatile int32_t txBuffer;
    volatile int32_t rxBuffer;

    public:

    SoftwareSerial(std::string, std::string);

    void begin(int);
    void setSpeed(int);
    void end(void);
    void setTX(void);
    void setRX(void);
    void setRXTX(bool);
    void send(void);
    void receive(void);
    void write(int);
    int16_t  read(void);
    bool listen(void);
    void tickerHandler(void);

    void enableTx(void);
    void enableRx(void);
    void idle() {__NOP();}
    

    int available();

    void flush_input_buffer();
    void printStr(char*);
};


#endif