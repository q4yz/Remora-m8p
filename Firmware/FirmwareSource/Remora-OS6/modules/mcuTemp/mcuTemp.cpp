#include "mcuTemp.h"

#ifdef MCU_TEMP_SUPPORTED
#include "stm32g0xx_ll_adc.h"

static bool pathsOn(void);
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
    faultCount(0),
    pending(0)
{
#ifdef MCU_TEMP_SUPPORTED
    this->sensor = new AnalogIn(ADC_TEMP);
    this->vref = new AnalogIn(ADC_VREF);   // the AnalogIn objects only initialise and calibrate the ADC

    pathsOn();
    wait_us(200);                           // temperature sensor start-up, 120 us

    bool valid;
    this->filtered = this->read(valid);
    mcuTemperature = this->filtered;
    printf("MCU temperature %.1f degC%s\n", this->filtered, valid ? "" : " (implausible)");
#endif
}


// one reading in degC, the supply (VDDA) measured with VREFINT: the factory calibration is at 3.0 V
#ifdef MCU_TEMP_SUPPORTED
// The internal channels are read straight from the registers. mbed's AnalogIn switches the sensor
// paths off after every read, and the HAL then busy-waits the 120 us temperature sensor start-up
// on the next one: a loop of ~500 us on the Cortex-M0+, in the servo interrupt, 10 times a second
// (servo thread overruns). Here the paths stay on; a thermistor read through AnalogIn still turns
// them off, pathsOn() then switches them on again and the reading waits for the next servo period.
static bool pathsOn(void)
{
    const uint32_t paths = LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT;
    uint32_t now = LL_ADC_GetCommonPathInternalCh(ADC1_COMMON);
    if ((now & paths) == paths) return true;
    LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, now | paths);
    return false;
}

// one conversion of an internal channel (CHSELR bit), 0 on failure. ~11 us: 160.5 + 12.5 ADC clocks at 16 MHz
static uint32_t convert(uint32_t chsel)
{
    ADC_TypeDef* adc = ADC1;
    if (!LL_ADC_IsEnabled(adc) || LL_ADC_REG_IsConversionOngoing(adc)) return 0;

    if (adc->CHSELR != chsel)
    {
        adc->ISR = ADC_ISR_CCRDY;
        adc->CHSELR = chsel;
        for (int i = 0; i < 1000 && !(adc->ISR & ADC_ISR_CCRDY); i++) {}
    }
    adc->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    LL_ADC_REG_StartConversion(adc);
    for (int i = 0; i < 2000; i++)
    {
        if (adc->ISR & ADC_ISR_EOC) return adc->DR & 0xFFF;
    }
    return 0;
}
#endif

float McuTemp::read(bool &valid)
{
    valid = false;
#ifdef MCU_TEMP_SUPPORTED
    uint32_t vrefRaw = convert(ADC_CHSELR_CHSEL13);         // VREFINT, 12 bit
    uint32_t tempRaw = convert(ADC_CHSELR_CHSEL12);         // temperature sensor
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
    // a reading postponed by slowUpdate() (sensor paths were off): update() runs right after
    // slowUpdate() in the same period, the reading is in the next one, >= 1 ms after switching on
    else if (this->pending && --this->pending == 0) this->slowUpdate();
}


void McuTemp::slowUpdate()
{
    if (mcuShutdown) return;            // latched until reset

#ifdef MCU_TEMP_SUPPORTED
    // switched off by a thermistor read: on now, read in the next servo period (sensor start-up)
    this->pending = pathsOn() ? 0 : 2;
    if (this->pending) return;
#endif

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
