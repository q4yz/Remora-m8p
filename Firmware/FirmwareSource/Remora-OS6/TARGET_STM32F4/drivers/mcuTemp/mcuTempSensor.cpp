#include "mcuTempSensor.h"

#include "mbed.h"
#include "stm32f4xx_ll_adc.h"

/***********************************************************************
    STM32F4 internal temperature sensor on ADC1 (channel 16 on the
    F40x / F41x, 18 on the F42x / F43x / F446, there shared with VBAT),
    the supply (VDDA) measured with VREFINT (channel 17): the factory
    calibration is at 3.3 V.

    Read straight from the registers. mbed's AnalogIn switches the
    sensor path (TSVREFE, common to the three ADCs) off after every read,
    of any channel: a thermistor read turns it off, mcuTempSensorReady()
    switches it on again and the reading waits for the next servo period
    (sensor start-up 10 us). Thermistors on ADC1 pins and this sensor are
    both read in the servo thread, never at once.
************************************************************************/

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx)
#define TEMP_CHANNEL        16
#else
#define TEMP_CHANNEL        18
#endif
#define VREF_CHANNEL        17


static bool pathsOn(void)
{
    if ((ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) == ADC_CCR_TSVREFE) return true;
    ADC->CCR = (ADC->CCR & ~ADC_CCR_VBATE) | ADC_CCR_TSVREFE;      // VBAT would take the temperature channel
    return false;
}


// one conversion of a channel, 0 on failure. 480 sampling cycles (>= 10 us for the sensor) +
// 15 at PCLK2 / 4 = 22.5 MHz: ~22 us
static uint32_t convert(uint32_t channel)
{
    ADC_TypeDef* adc = ADC1;
    if (!(adc->CR2 & ADC_CR2_ADON) || (adc->SR & ADC_SR_STRT && !(adc->SR & ADC_SR_EOC))) return 0;

    // the sampling time of the channel: mbed's AnalogIn may have set another for an ADC1 pin
    uint32_t shift = 3u * (channel - 10u);
    adc->SMPR1 = (adc->SMPR1 & ~(7u << shift)) | (7u << shift);    // 480 cycles
    adc->SQR1 &= ~ADC_SQR1_L;                                      // one conversion
    adc->SQR3 = channel;
    adc->SR = 0;
    adc->CR2 |= ADC_CR2_SWSTART;
    for (int i = 0; i < 20000; i++)
    {
        if (adc->SR & ADC_SR_EOC) return adc->DR & 0xFFF;
    }
    return 0;
}


void mcuTempSensorInit(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC_TypeDef* adc = ADC1;
    if (!(adc->CR2 & ADC_CR2_ADON))
    {
        // PCLK2 / 4 as mbed's AnalogIn sets it, 12 bit, right aligned, single conversion
        ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
        adc->CR1 = 0;
        adc->CR2 = ADC_CR2_ADON;
    }
    pathsOn();
    wait_us(20);                            // ADC and sensor start-up, 3 + 10 us
}


bool mcuTempSensorReady(void)
{
    return pathsOn();
}


bool mcuTempSensorRead(float* degC)
{
    uint32_t vrefRaw = convert(VREF_CHANNEL);
    uint32_t tempRaw = convert(TEMP_CHANNEL);
    if (vrefRaw == 0 || tempRaw == 0) return false;

    // the factory calibration (system memory, at 3.3 V): VREFINT at 30 degC, the sensor at 30 and 110 degC
    // (the LL helper macros are not defined for every F4 variant)
    const float vdda = 3300.0f * (float)*VREFINT_CAL_ADDR / (float)vrefRaw;
    if (vdda < 2500.0f || vdda > 3700.0f) return false;     // mV: not a usable reading

    const float cal1 = (float)*TEMPSENSOR_CAL1_ADDR, cal2 = (float)*TEMPSENSOR_CAL2_ADDR;
    if (cal2 <= cal1) return false;
    const float raw = (float)tempRaw * vdda / 3300.0f;      // as measured at the calibration supply
    *degC = (raw - cal1) * (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (cal2 - cal1)
            + (float)TEMPSENSOR_CAL1_TEMP;
    return true;
}
