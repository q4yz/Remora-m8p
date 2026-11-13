#include "canNodeIo.h"
#include "configuration.h"                  // board.h: BOARD_NODE_RESERVED_PINS

// Every STM32 family (REMORA_HAS_NODE_IO): GPIO by pin byte for the CAN nodes and the hub's TMC2209
// UART (modules/tmc/tmc2209Uart.cpp, the node's driver). The pin byte is the mbed PinName
// (port << 4 | number), GPIO through the family's HAL, the ADC through mbed, LED chains through the
// family's LedChain (if it has one).

#include "mbed.h"
#include "canProtocol.h"
#include "hal/analogin_api.h"
#include "hal/pinmap.h"
#include "PeripheralPins.h"
#ifdef REMORA_HAS_LED_CHAIN
#include "drivers/rgb/ledChain.h"
#endif

#ifdef REMORA_HAS_NODE_IO

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
#ifdef GPIOF
        case 5: return GPIOF;
#endif
#ifdef GPIOG
        case 6: return GPIOG;
#endif
#ifdef GPIOH
        case 7: return GPIOH;
#endif
    }
    return nullptr;
}


void canIoUniqueId(uint32_t id[3])
{
    id[0] = HAL_GetUIDw0();
    id[1] = HAL_GetUIDw1();
    id[2] = HAL_GetUIDw2();
}


bool canIoPinExists(uint8_t pin)
{
    return portOf(pin) != nullptr && CAN_PIN_NUMBER(pin) <= 15;
}


bool canIoPinReserved(uint8_t pin)
{
#ifdef BOARD_NODE_RESERVED_PINS
    // the pins the node board uses itself (board.h)
    static const uint8_t reserved[] = { BOARD_NODE_RESERVED_PINS };
    for (unsigned i = 0; i < sizeof(reserved); i++)
    {
        if (pin == reserved[i]) return true;
    }
#else
    (void)pin;
#endif
    return false;
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
#ifdef GPIOF
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
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


// WS2812 LED chains: CAN pin byte = mbed PinName (port << 4 | number) on the STM32
#ifdef REMORA_HAS_LED_CHAIN
#define RGB_CHAINS  CAN_NODE_RGB_CHAINS
static LedChain* rgbStrip[RGB_CHAINS];
static uint8_t rgbPin[RGB_CHAINS];

static int rgbSlot(uint8_t pin)
{
    for (int i = 0; i < RGB_CHAINS; i++) if (rgbStrip[i] && rgbPin[i] == pin) return i;
    return -1;
}


bool canIoRgbCapable(uint8_t pin)
{
    return canIoPinExists(pin) && ledChainCapable((PinName)pin);
}


bool canIoRgbConfigure(uint8_t pin, int leds, int bytesPerLed)
{
    canIoRgbRelease(pin);
    int i = 0;
    while (i < RGB_CHAINS && rgbStrip[i]) i++;
    if (i == RGB_CHAINS) return false;
    clockOn(portOf(pin));
    rgbStrip[i] = ledChainCreate((PinName)pin, leds, bytesPerLed, "RGB");
    rgbPin[i] = pin;
    return rgbStrip[i] != nullptr;
}


bool canIoRgbSend(uint8_t pin, const uint8_t* bytes)
{
    int i = rgbSlot(pin);
    return i >= 0 && rgbStrip[i]->send(bytes);
}


void canIoRgbRelease(uint8_t pin)
{
    int i = rgbSlot(pin);
    if (i < 0) return;
    rgbStrip[i]->release();
    delete rgbStrip[i];
    rgbStrip[i] = nullptr;
}
#else
// no LED chain driver in this family
bool canIoRgbCapable(uint8_t) { return false; }
bool canIoRgbConfigure(uint8_t, int, int) { return false; }
bool canIoRgbSend(uint8_t, const uint8_t*) { return false; }
void canIoRgbRelease(uint8_t) {}
#endif

#endif
