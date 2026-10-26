// The hub / standalone Remora program. CAN expansion nodes (REMORA_CAN_NODE, e.g. the EBB42)
// have their own main() in their target folder.
#ifndef REMORA_CAN_NODE


/*
Remora PRU firmware for LinuxCNC
Copyright (C) 2021  Scott Alford (scotta)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2
of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

// MBED includes
#include "mbed.h"
#include <cstdio>
#include <cerrno>
#include <string> 
#include "FATFileSystem.h"

#if defined TARGET_LPC176X || TARGET_STM32F1 || TARGET_SPIDER || TARGET_SPIDER_KING || TARGET_MONSTER8 || TARGET_ROBIN_3 || TARGET_MANTA8
#include "SDBlockDevice.h"
#elif defined TARGET_SKRV2 || TARGET_OCTOPUS_446 || TARGET_BLACK_F407VE || TARGET_OCTOPUS_429 | TARGET_SKRV3
#include "SDIOBlockDevice.h"
#endif

#include "configuration.h"
#include "remora.h"

#ifdef SD_UPLOAD
#include "sdUpload.h"
#endif

// libraries
#include "ArduinoJson.h"

// drivers
#include "RemoraComms.h"
#include "pin.h"
#include "softPwm.h"

// threads
#include "irqHandlers.h"
#include "interrupt.h"
#include "pruThread.h"
#include "createThreads.h"

// modules
#include "module.h"
#include "commsMonitor.h"
#include "blink.h"
#include "debug.h"
#include "digitalPin.h"
#include "encoder.h"
#include "eStop.h"
#include "hardwarePwm.h"
#include "mcp4451.h"
#include "motorPower.h"
#include "pwm.h"
#include "rcservo.h"
#include "resetPin.h"
#include "stepgen.h"

#if defined(TARGET_MANTA8)
// CAN expansion boards (CAN_PROTOCOL.md), the M8P is the hub
#define CAN_HUB
#include "CanHub.h"
#endif
#include "switch.h"
#include "temperature.h"
#include "tmc.h"
#include "qei.h"

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

// state machine
enum State {
    ST_SETUP = 0,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET
};

uint8_t resetCnt;
uint32_t base_freq = PRU_BASEFREQ;
uint32_t servo_freq = PRU_SERVOFREQ;

// boolean
volatile bool PRUreset;
bool configError = false;
bool threadsRunning = false;

// pointers to objects with global scope
pruThread* servoThread;
pruThread* baseThread;
pruThread* commsThread;

// unions for RX and TX data
//volatile rxData_t spiRxBuffer1;  // this buffer is used to check for valid data before moving it to rxData
//volatile rxData_t spiRxBuffer2;  // this buffer is used to check for valid data before moving it to rxData
volatile rxData_t rxData;
volatile txData_t txData;

// pointers to data
volatile rxData_t*  ptrRxData = &rxData;
volatile txData_t*  ptrTxData = &txData;
volatile int32_t* ptrTxHeader;  
volatile bool*    ptrPRUreset;
volatile int32_t* ptrJointFreqCmd[JOINTS];
volatile int32_t* ptrJointFeedback[JOINTS];
volatile uint8_t* ptrJointEnable;
volatile float*   ptrSetPoint[VARIABLES];
volatile float*   ptrProcessVariable[VARIABLES];
volatile uint16_t* ptrInputs;
volatile uint16_t* ptrOutputs;


/***********************************************************************
        OBJECTS etc                                           
************************************************************************/

// SD card access and Remora communication protocol
#if defined TARGET_SKRV1_4
    SDBlockDevice blockDevice(P0_9, P0_8, P0_7, P0_6);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData);

#elif defined TARGET_SKRV2 || TARGET_OCTOPUS_446 || TARGET_BLACK_F407VE || TARGET_OCTOPUS_429 || TARGET_SKRV3
    SDIOBlockDevice blockDevice;
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI1, PA_4);

#elif defined TARGET_MONSTER8
    SDBlockDevice blockDevice(PC_12, PC_11, PC_10, PC_9);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI1, PA_4);

#elif defined TARGET_ROBIN_3
    SDBlockDevice blockDevice(PC_12, PC_11, PC_10, PC_9);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI1, PE_10);  //use PE_10 as "slave select"

#elif defined TARGET_ROBIN_E3
    SDBlockDevice blockDevice(PB_15, PB_14, PB_13, PA_15);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI1, PA_4);

#elif defined TARGET_SKR_MINI_E3
    SDBlockDevice blockDevice(PA_7, PA_6, PA_5, PA_4);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI1, PC_1);    // use PC_1 as "slave select"

#elif defined TARGET_SPIDER
    SDBlockDevice blockDevice(PA_7, PA_6, PA_5, PA_4);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI1, PC_6);    // use PC_6 as "slave select"

#elif defined TARGET_SPIDER_KING
    SDBlockDevice blockDevice(PA_7, PA_6, PA_5, PA_4);  // mosi, miso, sclk, cs
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI2, PB_12);    // use PB_12 as "slave select" on SPI2

#elif defined TARGET_MANTA8
    // BTT Manta M8P V1.0 / V1.1: MCU SD card on SPI1 (shared with the TMC drivers), CS PA_8
    SDBlockDevice blockDevice(PA_7, PA_6, PA_5, PA_8);  // mosi, miso, sclk, cs
    // Raspberry Pi / CM4 link on SPI2 via the EXP2 header, hardware NSS on PB_12
    RemoraComms* comms = new RemoraComms(ptrRxData, ptrTxData, SPI2, PB_12);

#endif

// Watchdog
Watchdog& watchdog = Watchdog::get_instance();

// Json configuration file stuff
FATFileSystem fileSystem("fs");
FILE *jsonFile;
string strJson;
DynamicJsonDocument doc(JSON_BUFF_SIZE);
JsonObject thread;
JsonObject module;

#ifdef SD_UPLOAD
void stopThreads()
{
    if (threadsRunning)
    {
        baseThread->stopThread();
        servoThread->stopThread();
        threadsRunning = false;
    }
}

SDUpload sdUpload(comms, &fileSystem, &blockDevice, ptrTxData, stopThreads);
#endif

/***********************************************************************
        INTERRUPT HANDLERS - add NVIC_SetVector etc to setup()
************************************************************************/

// Add these to /thread/irqHandlers.h in the TARGET_target


/***********************************************************************
        ROUTINES
************************************************************************/

void readJsonConfig()
{
    printf("1. Reading json configuration file\n");

    // Try to mount the filesystem
    printf("Mounting the filesystem... ");
    fflush(stdout);
 
    int err = fileSystem.mount(&blockDevice);
    printf("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        printf("No filesystem found... ");
        fflush(stdout);
     }

    // Open the config file
    printf("Opening \"/fs/config.txt\"... ");
    fflush(stdout);
    jsonFile = fopen("/fs/config.txt", "r+");
    printf("%s\n", (!jsonFile ? "Fail :(" : "OK"));

    if (!jsonFile)
    {
        // keep going without modules so a config file can still be uploaded over SPI
        configError = true;
        return;
    }

    fseek (jsonFile, 0, SEEK_END);
    int32_t length = ftell (jsonFile);
    fseek (jsonFile, 0, SEEK_SET);

    printf("Json config file lenght = %2d\n", length);

    strJson.reserve(length+1);

    while (!feof(jsonFile)) {
        int c = fgetc(jsonFile);
        strJson.push_back(c);
    }

    // Remove comments from next line to print out the JSON config file
    //printf("%s\n", strJson.c_str());

    printf("\rClosing \"/fs/config.txt\"... ");
    fflush(stdout);
    fclose(jsonFile);
}


void setup()
{
    printf("\n2. Setting up DMA and threads\n");

    // TODO: we can probably just deinit the blockdevice for all targets....?

    #if defined TARGET_STM32F4
    // deinitialise the SDIO device to avoid DMA issues with the SPI DMA Slave on the STM32F4
    blockDevice.deinit();
    #endif

    #if defined TARGET_SKR_MINI_E3
    // remove the SD device as we are sharing the SPI with the comms module
    blockDevice.deinit();
    #endif

    #if defined TARGET_MANTA8
    // release the SD card, its SPI bus is shared with the TMC drivers
    blockDevice.deinit();
    #endif

    // initialise the Remora comms 
    comms->init();
    comms->start();
}


void deserialiseJSON()
{
    printf("\n3. Parsing json configuration file\n");

    const char *json = strJson.c_str();

    // parse the json configuration file
    DeserializationError error = deserializeJson(doc, json);

    printf("Config deserialisation - ");

    switch (error.code())
    {
        case DeserializationError::Ok:
            printf("Deserialization succeeded\n");
            break;
        case DeserializationError::InvalidInput:
            printf("Invalid input!\n");
            configError = true;
            break;
        case DeserializationError::NoMemory:
            printf("Not enough memory\n");
            configError = true;
            break;
        default:
            printf("Deserialization failed\n");
            configError = true;
            break;
    }
}


#ifdef CAN_HUB
RemoraCAN* canBus = nullptr;
CanHub* canHub = nullptr;

void configCan()
{
    // optional: "CAN": { "Bitrate": 1000000, "Nodes": [ { "UUID": "...", "Node": 1, "Comment": "..." } ] }
    if (configError || !doc.containsKey("CAN")) return;

    printf("\n4b. Config CAN\n");

    JsonObject cfg = doc["CAN"];
    uint32_t bitrate = cfg["Bitrate"] | 1000000;

    // Manta M8P: FDCAN1 on PD12 (RX) / PD13 (TX), alternate function 3, on-board transceiver
    canBus = new RemoraCAN(FDCAN1, GPIOD, 12, 13, GPIO_AF3_FDCAN1, bitrate);
    if (!canBus->start()) return;

    canHub = new CanHub(canBus);

    JsonArray nodes = cfg["Nodes"];
    for (JsonArray::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        JsonObject n = *it;
        canHub->addNode(n["UUID"] | "", n["Node"] | 0, n["Comment"] | "");
    }
}
#endif

void configThreads()
{
    if (configError) return;

    printf("\n4. Config threads\n");

    JsonArray Threads = doc["Threads"];

    // create objects from json data
    for (JsonArray::iterator it=Threads.begin(); it!=Threads.end(); ++it)
    {
        thread = *it;
        
        const char* configor = thread["Thread"];
        uint32_t    freq = thread["Frequency"];

        if (!strcmp(configor,"Base"))
        {
            base_freq = freq;
            printf("Setting BASE thread frequency to %d\n", base_freq);
        }
        else if (!strcmp(configor,"Servo"))
        {
            servo_freq = freq;
            printf("Setting SERVO thread frequency to %d\n", servo_freq);
        }
    }
}


void loadModules()
{
    if (configError) return;

    printf("\n5. Loading modules\n");

    // SPI communication monitoring
    servoThread->registerModule(comms);

    // outputs go to their safe state while there is no SPI data from LinuxCNC
    servoThread->registerModule(new CommsMonitor(comms));

    JsonArray Modules = doc["Modules"];

    // create objects from json data
    for (JsonArray::iterator it=Modules.begin(); it!=Modules.end(); ++it)
    {
        module = *it;
        
        const char* thread = module["Thread"];
        const char* type = module["Type"];

        if (!strcmp(thread,"Base"))
        {
            printf("\nBase thread object\n");

            if (!strcmp(type,"Stepgen"))
            {
                createStepgen();
            }
            else if (!strcmp(type,"Encoder"))
            {
                createEncoder();
            }
            else if (!strcmp(type,"RCServo"))
            {
                createRCServo();
            }
        }
        else if (!strcmp(thread,"Servo"))
        {
            printf("\nServo thread object\n");

            if (!strcmp(type, "eStop"))
            {
                createEStop();
            }
            else if (!strcmp(type, "Reset Pin"))
            {
                createResetPin();
            }
            else if (!strcmp(type, "Blink"))
            {
                createBlink();
            }
            else if (!strcmp(type,"Digital Pin"))
            {
                createDigitalPin();
            }
            else if (!strcmp(type,"PWM"))
            {
                createPWM();
            }
            else if (!strcmp(type,"Temperature"))
            { 
                createTemperature();
            }
            else if (!strcmp(type,"Switch"))
            {
                createSwitch();
            }
            else if (!strcmp(type,"QEI"))
            {
                createQEI();
            }
        }
        else if (!strcmp(thread,"On load"))
        {
            printf("\nOn load - run once module\n");


            if (!strcmp(type,"MCP4451")) // digipot
            {
				createMCP4451();
            }
            else if (!strcmp(type,"Motor Power"))
            {
                createMotorPower();
            }
            else if (!strcmp(type,"TMC2208"))
            {
                createTMC2208();
            }
            else if (!strcmp(type,"TMC2209"))
            {
                createTMC2209();
            }
        }
    }
}


void debugThreadHigh()
{
    //Module* debugOnB = new Debug("PC_1", 1);
    //baseThread->registerModule(debugOnB);

    //Module* debugOnS = new Debug("PC_3", 1);
    //servoThread->registerModule(debugOnS);

    //Module* debugOnC = new Debug("PE_6", 1);
    //commsThread->registerModule(debugOnC);
}

void debugThreadLow()
{
    //Module* debugOffB = new Debug("PC_1", 0);
    //baseThread->registerModule(debugOffB); 

    //Module* debugOffS = new Debug("PC_3", 0);
    //servoThread->registerModule(debugOffS);

    //commsThread->startThread();
    //Module* debugOffC = new Debug("PE_6", 0);
    //commsThread->registerModule(debugOffC); 
}

#if REMORA_DEBUG
// Debug output on the console UART (Manta M8P: EXP2 pin 7, 115200 baud)

#if defined(TARGET_MANTA8)
#include "rawConsole.h"     // register level console, replaces the mbed serial console in debug builds
#endif

void debugBootInfo()
{
    printf("[debug] SystemCoreClock %lu Hz, reset cause:", (unsigned long)SystemCoreClock);
#ifdef RCC_FLAG_PORRST
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) printf(" power-on");
#endif
#ifdef RCC_FLAG_PWRRST
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST)) printf(" power-on");
#endif
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) printf(" reset-pin");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) printf(" software");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) printf(" WATCHDOG");
    printf("\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void debugLinkReport(int state)
{
    // printf blocks the main loop (~87 us per character), so only report while the link is not
    // running, a report while running would make the board miss SPI frames and drop the link
    static const char* const names[] = {"SETUP", "START", "IDLE", "RUNNING", "STOP", "RESET", "WDRESET"};
    static uint32_t last = 0;

    if (state == ST_RUNNING || (HAL_GetTick() - last) < 1000) return;
    last = HAL_GetTick();

    printf("[debug] %s reset-pin=%d frames=%lu read=%lu write=%lu rejected=%lu (last 0x%08lx) dma-timeouts=%lu config=%s\n",
           names[state], (int)PRUreset,
           (unsigned long)comms->dbgFrames, (unsigned long)comms->dbgReads, (unsigned long)comms->dbgWrites,
           (unsigned long)comms->dbgRejects, (unsigned long)comms->dbgLastBad, (unsigned long)comms->dbgTimeouts,
           configError ? "ERROR" : "ok");
}
#endif

int main()
{
    
    enum State currentState;
    enum State prevState;

#if REMORA_DEBUG && defined(TARGET_MANTA8)
    rawConsolePuts("[raw] main\n");
#endif

    comms->setStatus(false);
    comms->setError(false);
    currentState = ST_SETUP;
    prevState = ST_RESET;

    printf("\nRemora PRU - Programmable Realtime Unit \n");
    printf("\n Mbed-OS6 \n");
    printf("\n Remora-spi Driver \n");

#if REMORA_DEBUG
    debugBootInfo();
#endif

    watchdog.start(2000);

    while(1)
    {
      // the main loop does very little, keeping the Watchdog serviced and
      // resetting the rxData buffer if there is a loss of SPI commmunication
      // with LinuxCNC. Everything else is done via DMA and within the
      // two threads- Base and Servo threads that run the Modules.

    watchdog.kick();

#ifdef SD_UPLOAD
    // SD card upload from the Raspberry Pi while LinuxCNC is offline, suspends normal operation
    sdUpload.service();
    if (sdUpload.isActive())
    {
        comms->SPItasks();
        continue;
    }
#endif

#ifdef CAN_HUB
    // CAN expansion nodes; silent while the SPI link runs (blocking printf would cost frames)
    if (canHub) canHub->service(HAL_GetTick(), currentState == ST_RUNNING);
#endif

#if REMORA_DEBUG
    debugLinkReport(currentState);
#if defined(TARGET_MANTA8)
    rawHeartbeat();
#endif
#endif

    switch(currentState){
        case ST_SETUP:
            // do setup tasks
            if (currentState != prevState)
            {
                printf("\n## Entering SETUP state\n");
            }
            prevState = currentState;

            readJsonConfig();
            setup();
            deserialiseJSON();
#ifdef CAN_HUB
            configCan();
#endif
            configThreads();
            createThreads();
            //debugThreadHigh();
            loadModules();
            //debugThreadLow();

            currentState = ST_START;
            break; 

        case ST_START:
            // do start tasks
            if (currentState != prevState)
            {
                printf("\n## Entering START state\n");
            }
            prevState = currentState;

            if (!threadsRunning)
            {
                // Start the threads
                printf("\nStarting the BASE thread\n");
                baseThread->startThread();
                
                printf("\nStarting the SERVO thread\n");
                servoThread->startThread();

                threadsRunning = true;

                // wait for threads to read IO before testing for PRUreset
                //wait(1);
                //ThisThread::sleep_for(100);
                wait_us(1000000);
            }

            if (PRUreset)
            {
                // RPi outputs default is high until configured when LinuxCNC spiPRU component is started, PRUreset pin will be high
                // stay in start state until LinuxCNC is started
                currentState = ST_START;
            }
            else
            {
                currentState = ST_IDLE;
            }
            
            break;


        case ST_IDLE:
            // do something when idle
            if (currentState != prevState)
            {
                printf("\n## Entering IDLE state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors
            if (comms->getError())
            {
                printf("Communication data error\n");
                comms->setError(false);
            }

            //wait for SPI data before changing to running state
            if (comms->getStatus())
            {
                currentState = ST_RUNNING;
            }

            if (PRUreset) 
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_RUNNING:
            // do running tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RUNNING state\n");
            }
            prevState = currentState;

            if (comms->getStatus() == false)
            {
                currentState = ST_RESET;
            }

            if (PRUreset) 
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_STOP:
            // do stop tasks
            if (currentState != prevState)
            {
                printf("\n## Entering STOP state\n");
            }
            prevState = currentState;


            currentState = ST_STOP;
            break;

        case ST_RESET:
            // do reset tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RESET state\n");
            }
            prevState = currentState;

            // set all of the rxData buffer to 0
            // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
            printf("   Resetting rxBuffer\n");
            {
                int n = sizeof(rxData.rxBuffer);
                while(n-- > 0)
                {
                    rxData.rxBuffer[n] = 0;
                }
            }

            currentState = ST_IDLE;
            break;

        case ST_WDRESET:
            // do a watch dog reset
            printf("\n## Entering WDRESET state\n");

            // force a watchdog reset by looping here
            while(1){}

            break;
      }

    comms->SPItasks();

    //ThisThread::sleep_for(LOOP_TIME);
    //wait(LOOP_TIME);
    //wait_us(LOOP_TIME * 1000000);

    }
}

#endif // REMORA_CAN_NODE
