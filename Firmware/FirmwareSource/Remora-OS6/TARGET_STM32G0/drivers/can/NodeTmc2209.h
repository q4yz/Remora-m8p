#ifndef NODETMC2209_H
#define NODETMC2209_H

#include <stdint.h>

/***********************************************************************
    TMC2209 stepper driver of a CAN node, configured over its single wire
    UART (PDN_UART, e.g. PA15 on the EBB42).

    The UART is bit-banged in the node base thread: tick() runs at
    CAN_NODE_BASE_FREQ, one bit is TMC_OVERSAMPLE ticks (40 kHz / 3 = 13333
    baud, the TMC2209 measures the baud rate from the sync nibble of every
    datagram). service() runs in the main loop and never blocks:

    - configure: read IFCNT, write the registers, read IFCNT again; the
      count must have gone up by the number of writes, else retry
    - then GSTAT every TMC_CHECK_MS: a driver reset (motor power lost) or
      no answer -> configure again

    Current, microsteps and chopper as the Remora TMC2209 module, but with
    the internal current reference (I_scale_analog 0): VREF is not used.
************************************************************************/

#define TMC_OVERSAMPLE          3           // base ticks per bit
#define TMC_CHECK_MS            1000        // GSTAT check / retry period
#define TMC_RETRIES             3           // per datagram

// driver status, reported to the hub in EMCY
#define TMC_STATUS_NONE         0           // not configured yet
#define TMC_STATUS_OK           1           // configured, checked
#define TMC_STATUS_NO_ANSWER    2           // no / corrupt reply: no motor power, wrong pin or address
#define TMC_STATUS_WRITE_FAILED 3           // replies, but the writes did not arrive (IFCNT)

// the registers written by a configuration
#define TMC_CONFIG_REGS         8

class NodeTmc2209
{
    private:

        // UART, shared with the interrupt: the main loop starts a transfer, the interrupt finishes it
        uint8_t           pin;
        volatile bool     active;
        volatile uint8_t  uart;             // UART_*
        uint8_t           txBuf[8];
        uint8_t           txLen;
        uint8_t           rxBuf[8];
        uint8_t           rxLen;            // bytes expected
        volatile uint8_t  rxCount;          // bytes received
        uint8_t           byteIndex;        // interrupt: byte being sent
        int8_t            bitIndex;         // interrupt: bit being sent / received, -1: start bit
        uint8_t           tickCount;        // interrupt: ticks left of the current bit
        uint16_t          shift;            // interrupt: frame being sent / received
        uint16_t          timeout;          // interrupt: ticks left to the first / next start bit

        // registers, computed by the constructor
        uint8_t           address;
        uint8_t           regAddr[TMC_CONFIG_REGS];
        uint32_t          regValue[TMC_CONFIG_REGS];

        // main loop state machine
        uint8_t           step;             // STEP_*
        uint8_t           regIndex;
        uint8_t           retries;
        uint8_t           ifcnt;            // IFCNT before the writes
        uint32_t          readValue;
        uint32_t          now;
        uint32_t          wait;             // ms, next action not before this
        bool              firstService;
        uint8_t           status;           // TMC_STATUS_*
        uint8_t           version;          // IOIN version, 0x21
        uint32_t          configs;          // configurations done (debug)

        void startTransfer(uint8_t len, uint8_t replyLen);
        void sendWrite(uint8_t reg, uint32_t value);
        void sendRead(uint8_t reg);
        bool readReply(uint8_t reg);
        void done(uint8_t status);

    public:

        // rsense in mOhm, current in mA (RMS), microsteps 1 ... 256, stall: SGTHRS, 0 off
        void setup(uint8_t pin, uint8_t address, uint16_t rsense, uint16_t current, uint16_t microsteps,
                   bool stealth, uint8_t stall);
        void start(void);                   // pin set up by the caller, configure now
        void stop(void);                    // the interrupt no longer uses the pin

        void tick(void);                    // base thread interrupt
        bool service(uint32_t now);         // main loop; true: status changed

        uint8_t getStatus(void) { return this->status; }
        uint8_t getVersion(void) { return this->version; }
        uint8_t getPin(void) { return this->pin; }
        uint32_t getConfigs(void) { return this->configs; }
        uint32_t getRegister(int i) { return this->regValue[i]; }       // test
        uint8_t getRegisterAddress(int i) { return this->regAddr[i]; }   // test
};

uint8_t tmcCrc(const uint8_t* data, int len);
const char* tmcStatusText(uint8_t status);

#endif
