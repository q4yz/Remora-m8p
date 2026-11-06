#include "mcuTemp.h"

#ifdef MCU_TEMP_SUPPORTED
#include "stm32g0xx_ll_adc.h"
#endif

volatile bool mcuShutdown = false;
volatile float mcuTemperature = 0.0f;

static McuTemp* guard = nullptr;
static volatile bool warned = false;    // MCU_TEMP_WARNING reached, for the console
static volatile bool sensorFault = false;


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

void startMcuTempGuard()
{
#ifdef MCU_TEMP_SUPPORTED
    if (guard) return;
    printf("MCU over temperature shutdown at %.0f degC\n", MCU_TEMP_SHUTDOWN);
    guard = new McuTemp(txData.header);
    servoThread->registerModule(guard);
#endif
}


void createMcuTemperature()
{
    const char* comment = module["Comment"] | "MCU";
    int pv = module["PV[i]"] | -1;
    printf("%s\n", comment);

#ifdef MCU_TEMP_SUPPORTED
    if (pv < 0 || pv >= VARIABLES)
    {
        printf("MCU Temperature: PV[i] %d out of range 0 ... %d\n", pv, VARIABLES - 1);
        return;
    }
    startMcuTempGuard();
    ptrProcessVariable[pv] = &txData.processVariable[pv];
    guard->setFeedback(ptrProcessVariable[pv]);
    printf("MCU temperature to LinuxCNC PV[%d]\n", pv);
#else
    printf("MCU Temperature: not supported on this board\n");
#endif
}


void reportMcuTemp()
{
    static bool shutdownReported = false, warningReported = false;

    if (mcuShutdown && !shutdownReported)
    {
        shutdownReported = true;
        if (sensorFault)
            printf("\nMCU SHUTDOWN: no plausible reading from the MCU temperature sensor, outputs off until reset\n");
        else
            printf("\nMCU SHUTDOWN: MCU at %.1f degC (limit %.0f degC), outputs off until reset\n",
                   mcuTemperature, MCU_TEMP_SHUTDOWN);
    }
    else if (warned && !warningReported)
    {
        warningReported = true;
        printf("\nWARNING: MCU at %.1f degC, shutdown at %.0f degC\n", mcuTemperature, MCU_TEMP_SHUTDOWN);
    }
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

McuTemp::McuTemp(volatile int32_t &ptrTxHeader) :
    Module(servo_freq, MCU_TEMP_UPDATE_HZ),
    ptrFeedback(nullptr),
    ptrTxHeader(&ptrTxHeader),
    filtered(0.0f),
    overCount(0),
    faultCount(0)
{
#ifdef MCU_TEMP_SUPPORTED
    this->sensor = new AnalogIn(ADC_TEMP);
    this->vref = new AnalogIn(ADC_VREF);

    bool valid;
    this->filtered = this->read(valid);
    mcuTemperature = this->filtered;
    printf("MCU temperature %.1f degC%s\n", this->filtered, valid ? "" : " (implausible)");
#endif
}


// one reading in degC, the supply (VDDA) measured with VREFINT: the factory calibration is at 3.0 V
float McuTemp::read(bool &valid)
{
    valid = false;
#ifdef MCU_TEMP_SUPPORTED
    uint32_t vrefRaw = this->vref->read_u16() >> 4;         // 12 bit
    uint32_t tempRaw = this->sensor->read_u16() >> 4;
    if (vrefRaw == 0 || tempRaw == 0) return 0.0f;

    uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefRaw, LL_ADC_RESOLUTION_12B);
    if (vdda < 2500 || vdda > 3700) return 0.0f;            // mV: not a usable reading

    float t = (float)__LL_ADC_CALC_TEMPERATURE(vdda, tempRaw, LL_ADC_RESOLUTION_12B);
    if (t < -40.0f || t > 150.0f) return t;

    valid = true;
    return t;
#else
    return 0.0f;
#endif
}


void McuTemp::update()
{
    // every servo period: the shutdown header wins over the eStop module and the comms default
    if (mcuShutdown) *this->ptrTxHeader = PRU_OVERTEMP;
}


void McuTemp::slowUpdate()
{
    if (mcuShutdown) return;            // latched until reset

    bool valid;
    float t = this->read(valid);

    if (!valid)
    {
        if (++this->faultCount >= MCU_TEMP_FAULT_READINGS)
        {
            sensorFault = true;
            mcuShutdown = true;
            *this->ptrTxHeader = PRU_OVERTEMP;
        }
        return;
    }
    this->faultCount = 0;

    // light filtering against ADC noise, a single reading over the limit does not trip
    this->filtered += (t - this->filtered) * 0.5f;
    mcuTemperature = this->filtered;
    if (this->ptrFeedback) *this->ptrFeedback = this->filtered;

    if (this->filtered >= MCU_TEMP_WARNING) warned = true;

    if (this->filtered >= MCU_TEMP_SHUTDOWN)
    {
        if (++this->overCount >= MCU_TEMP_TRIP_READINGS)
        {
            mcuShutdown = true;
            *this->ptrTxHeader = PRU_OVERTEMP;
        }
    }
    else
    {
        this->overCount = 0;
    }
}
