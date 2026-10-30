#include "canNodeIo.h"

// CAN nodes only, the hub uses the Remora modules for its own pins
#ifdef REMORA_CAN_NODE

#include "stm32g0xx_hal.h"
#include "canProtocol.h"
#include "hal/analogin_api.h"
#include "hal/pinmap.h"
#include "PeripheralPins.h"

static GPIO_TypeDef* portOf(uint8_t pin)
{
    switch (CAN_PIN_PORT(pin))
    {
        case 0: return GPIOA;
        case 1: return GPIOB;
        case 2: return GPIOC;
        case 3: return GPIOD;
#ifdef GPIOE
        case 4: return GPIOE;
#endif
        case 5: return GPIOF;
    }
    return nullptr;
}


bool canIoPinExists(uint8_t pin)
{
    return portOf(pin) != nullptr && CAN_PIN_NUMBER(pin) <= 15;
}


bool canIoPinReserved(uint8_t pin)
{
    // EBB42: CAN on PB0 / PB1 (FDCAN2), USB on PA11 / PA12
    return pin == CAN_PIN(1, 0) || pin == CAN_PIN(1, 1) ||
           pin == CAN_PIN(0, 11) || pin == CAN_PIN(0, 12);
}


static void clockOn(GPIO_TypeDef* port)
{
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
#ifdef GPIOE
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
}


void canIoConfigure(uint8_t pin, bool output, uint8_t modifier, bool level)
{
    GPIO_TypeDef* port = portOf(pin);
    if (!port) return;
    clockOn(port);

    uint16_t mask = (uint16_t)(1u << CAN_PIN_NUMBER(pin));
    if (output) HAL_GPIO_WritePin(port, mask, level ? GPIO_PIN_SET : GPIO_PIN_RESET);     // no glitch when it turns on

    GPIO_InitTypeDef init = {0};
    init.Pin = mask;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Pull = (modifier == DPIN_MOD_PULLUP) ? GPIO_PULLUP : (modifier == DPIN_MOD_PULLDOWN) ? GPIO_PULLDOWN : GPIO_NOPULL;
    if (output) init.Mode = (modifier == DPIN_MOD_OPENDRAIN) ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
    else init.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(port, &init);
}


void canIoRelease(uint8_t pin)
{
    GPIO_TypeDef* port = portOf(pin);
    if (port) HAL_GPIO_DeInit(port, (uint16_t)(1u << CAN_PIN_NUMBER(pin)));
}


void canIoWrite(uint8_t pin, bool level)
{
    GPIO_TypeDef* port = portOf(pin);
    if (port) HAL_GPIO_WritePin(port, (uint16_t)(1u << CAN_PIN_NUMBER(pin)), level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


bool canIoRead(uint8_t pin)
{
    GPIO_TypeDef* port = portOf(pin);
    return port && HAL_GPIO_ReadPin(port, (uint16_t)(1u << CAN_PIN_NUMBER(pin))) == GPIO_PIN_SET;
}


void canIoSetOutput(uint8_t pin, bool output)
{
    GPIO_TypeDef* port = portOf(pin);
    if (!port) return;
    uint32_t shift = 2u * CAN_PIN_NUMBER(pin);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    port->MODER = (port->MODER & ~(3u << shift)) | ((output ? 1u : 0u) << shift);     // 01 output, 00 input
    __set_PRIMASK(primask);
}


// the pin byte is the STM32 PinName (port << 4 | number)
#define ANALOG_PINS 4                       // CAN_NODE_ANALOG
static analogin_t analogIn[ANALOG_PINS];
static uint8_t analogPin[ANALOG_PINS];
static int analogUsed = 0;

bool canIoAnalogCapable(uint8_t pin)
{
    return canIoPinExists(pin) && pinmap_find_peripheral((PinName)pin, PinMap_ADC) != (uint32_t)NC;
}


static analogin_t* analogOf(uint8_t pin)
{
    for (int i = 0; i < analogUsed; i++) if (analogPin[i] == pin) return &analogIn[i];
    return nullptr;
}


void canIoAnalogConfigure(uint8_t pin)
{
    analogin_t* a = analogOf(pin);
    if (!a)
    {
        if (analogUsed >= ANALOG_PINS) return;
        analogPin[analogUsed] = pin;
        a = &analogIn[analogUsed++];
    }
    analogin_init(a, (PinName)pin);
}


uint16_t canIoAnalogRead(uint8_t pin)
{
    analogin_t* a = analogOf(pin);
    return a ? analogin_read_u16(a) : 0;
}

#endif
