#ifndef MCUTEMPSENSOR_H
#define MCUTEMPSENSOR_H

/***********************************************************************
    The MCU's internal temperature sensor, for the over temperature
    shutdown (modules/mcuTemp)

    Read in the servo thread, 10 times a second, so a reading must take
    a few us, not the HAL's sensor start-up wait. A family with
    REMORA_HAS_MCU_TEMP implements it (STM32G0:
    TARGET_STM32G0/drivers/mcuTemp, the ADC registers) and sets the
    limits of its MCU in family.h (MCU_TEMP_SHUTDOWN, MCU_TEMP_WARNING).
************************************************************************/

// once, from the main loop: ADC and sensor on, may wait for the sensor to start up
void mcuTempSensorInit(void);
// servo thread: false when the sensor had to be switched on again (something else used the ADC and
// switched it off): read not before the next servo period
bool mcuTempSensorReady(void);
// servo thread: one reading in degC; false: no usable reading (ADC busy, supply out of range).
// A value outside -40 ... 150 degC counts as implausible too
bool mcuTempSensorRead(float* degC);

#endif
