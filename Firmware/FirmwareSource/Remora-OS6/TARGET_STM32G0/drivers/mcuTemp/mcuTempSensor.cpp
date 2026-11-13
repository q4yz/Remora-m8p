#include "mcuTempSensor.h"

#include "mbed.h"
#include "stm32g0xx_ll_adc.h"

/***********************************************************************
    STM32G0 internal temperature sensor, the supply (VDDA) measured with
    VREFINT: the factory calibration is at 3.0 V
************************************************************************/

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


void mcuTempSensorInit(void)
{
    // the AnalogIn objects only initialise and calibrate the ADC
    static AnalogIn sensor(ADC_TEMP);
    static AnalogIn vref(ADC_VREF);

    pathsOn();
    wait_us(200);                           // temperature sensor start-up, 120 us
}


bool mcuTempSensorReady(void)
{
    return pathsOn();
}


bool mcuTempSensorRead(float* degC)
{
    uint32_t vrefRaw = convert(ADC_CHSELR_CHSEL13);         // VREFINT, 12 bit
    uint32_t tempRaw = convert(ADC_CHSELR_CHSEL12);         // temperature sensor
    if (vrefRaw == 0 || tempRaw == 0) return false;

    uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefRaw, LL_ADC_RESOLUTION_12B);
    if (vdda < 2500 || vdda > 3700) return false;           // mV: not a usable reading

    *degC = (float)__LL_ADC_CALC_TEMPERATURE(vdda, tempRaw, LL_ADC_RESOLUTION_12B);
    return true;
}
