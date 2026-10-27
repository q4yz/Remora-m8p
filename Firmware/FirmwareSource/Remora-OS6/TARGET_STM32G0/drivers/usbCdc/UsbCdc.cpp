#include "UsbCdc.h"

// CAN nodes only: the hub (M8P) talks SPI and has no use for the USB port
#ifdef REMORA_CAN_NODE

#include <string.h>

#define EP0_SIZE        64
#define EP_NOTIFY       0x81        // interrupt IN, CDC notifications (never sent)
#define EP_DATA_OUT     0x02        // bulk OUT, discarded
#define EP_DATA_IN      0x82        // bulk IN, debug output
#define DATA_SIZE       64

// packet memory: buffer descriptor table (8 channels x 8 bytes) at 0, then the buffers
#define PMA_EP0_OUT     0x040
#define PMA_EP0_IN      0x080
#define PMA_NOTIFY      0x0C0
#define PMA_DATA_OUT    0x0D0
#define PMA_DATA_IN     0x110

UsbCdc usbCdc;

static const uint8_t deviceDescriptor[18] =
{
    18, 0x01,                   // device
    0x00, 0x02,                 // USB 2.0
    0x02, 0x00, 0x00,           // class CDC
    EP0_SIZE,
    0x83, 0x04,                 // VID 0x0483 (ST)
    0x40, 0x57,                 // PID 0x5740 (ST virtual COM port)
    0x00, 0x01,                 // device release 1.00
    1, 2, 3,                    // manufacturer, product, serial number strings
    1                           // configurations
};

static const uint8_t configDescriptor[67] =
{
    9, 0x02, 67, 0, 2, 1, 0, 0x80, 50,          // configuration: 2 interfaces, bus powered, 100 mA

    9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,      // interface 0: CDC communication, ACM, AT commands
    5, 0x24, 0x00, 0x10, 0x01,                  // header functional, CDC 1.10
    5, 0x24, 0x01, 0x00, 0x01,                  // call management: data interface 1
    4, 0x24, 0x02, 0x02,                        // ACM: line coding and control line state
    5, 0x24, 0x06, 0, 1,                        // union: master 0, slave 1
    7, 0x05, EP_NOTIFY, 0x03, 8, 0, 16,         // interrupt IN, 8 bytes, 16 ms

    9, 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0,      // interface 1: CDC data
    7, 0x05, EP_DATA_OUT, 0x02, DATA_SIZE, 0, 0,
    7, 0x05, EP_DATA_IN, 0x02, DATA_SIZE, 0, 0
};

static const uint8_t languageDescriptor[4] = { 4, 0x03, 0x09, 0x04 };     // English (US)


UsbCdc::UsbCdc() :
    ep0Data(nullptr),
    ep0Remaining(0),
    ep0Zlp(false),
    ep0InData(false),
    ep0LineCoding(false),
    configured(false),
    txBusy(false),
    txHead(0),
    txTail(0)
{
    static const uint8_t defaultCoding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };   // 115200 8N1
    memcpy(this->lineCoding, defaultCoding, sizeof(this->lineCoding));
    memset(&this->hpcd, 0, sizeof(this->hpcd));
    this->serial[0] = 0;
}


void UsbCdc::start(const char* serialNumber)
{
    strncpy(this->serial, serialNumber ? serialNumber : "", sizeof(this->serial) - 1);
    this->serial[sizeof(this->serial) - 1] = 0;

    // USB clock: HSI48, kept on 48 MHz by the clock recovery system from the USB start of frame
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&clk);

    __HAL_RCC_CRS_CLK_ENABLE();
    RCC_CRSInitTypeDef crs = {0};
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    HAL_RCCEx_CRSConfig(&crs);

    // USB transceiver supply, PA11 / PA12 need no pin setup
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddUSB();
    __HAL_RCC_USB_CLK_ENABLE();

    // HAL_PCD_IRQHandler() first checks the interrupt line status in SYSCFG (ITLINE8),
    // which reads 0 without the SYSCFG clock: every USB event would be ignored
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    this->hpcd.Instance = USB_DRD_FS;
    this->hpcd.Init.dev_endpoints = 8;
    this->hpcd.Init.speed = PCD_SPEED_FULL;
    this->hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
    this->hpcd.Init.Sof_enable = DISABLE;
    this->hpcd.Init.low_power_enable = DISABLE;
    this->hpcd.Init.lpm_enable = DISABLE;
    this->hpcd.Init.battery_charging_enable = DISABLE;
    this->hpcd.Init.vbus_sensing_enable = DISABLE;
    this->hpcd.Init.bulk_doublebuffer_enable = DISABLE;
    this->hpcd.Init.iso_singlebuffer_enable = DISABLE;
    if (HAL_PCD_Init(&this->hpcd) != HAL_OK) return;

    HAL_PCDEx_PMAConfig(&this->hpcd, 0x00, PCD_SNG_BUF, PMA_EP0_OUT);
    HAL_PCDEx_PMAConfig(&this->hpcd, 0x80, PCD_SNG_BUF, PMA_EP0_IN);
    HAL_PCDEx_PMAConfig(&this->hpcd, EP_NOTIFY, PCD_SNG_BUF, PMA_NOTIFY);
    HAL_PCDEx_PMAConfig(&this->hpcd, EP_DATA_OUT, PCD_SNG_BUF, PMA_DATA_OUT);
    HAL_PCDEx_PMAConfig(&this->hpcd, EP_DATA_IN, PCD_SNG_BUF, PMA_DATA_IN);

    HAL_PCD_Start(&this->hpcd);         // pull-up on: the host enumerates the device
}


void UsbCdc::service()
{
    if (!this->hpcd.Instance) return;

    // the interrupt handler, polled: the USB interrupt stays disabled in the NVIC
    HAL_PCD_IRQHandler(&this->hpcd);

    if (!this->configured || this->txBusy || this->txHead == this->txTail) return;

    // at most 63 bytes: a short packet ends the transfer, no zero length packet needed
    uint16_t n = 0;
    while (n < DATA_SIZE - 1 && this->txTail != this->txHead)
    {
        this->txPacket[n++] = this->txBuffer[this->txTail];
        this->txTail = (this->txTail + 1) % USBCDC_TX_BUFFER;
    }
    this->txBusy = true;
    HAL_PCD_EP_Transmit(&this->hpcd, EP_DATA_IN, this->txPacket, n);
}


int UsbCdc::write(const void* data, int len)
{
    const uint8_t* p = (const uint8_t*)data;
    for (int i = 0; i < len; i++)
    {
        uint16_t next = (this->txHead + 1) % USBCDC_TX_BUFFER;
        if (next == this->txTail) break;                // full: drop the rest
        this->txBuffer[this->txHead] = p[i];
        this->txHead = next;
    }
    return len;
}


void UsbCdc::openEndpoints()
{
    HAL_PCD_EP_Open(&this->hpcd, EP_NOTIFY, 8, EP_TYPE_INTR);
    HAL_PCD_EP_Open(&this->hpcd, EP_DATA_OUT, DATA_SIZE, EP_TYPE_BULK);
    HAL_PCD_EP_Open(&this->hpcd, EP_DATA_IN, DATA_SIZE, EP_TYPE_BULK);
    HAL_PCD_EP_Receive(&this->hpcd, EP_DATA_OUT, this->rxPacket, DATA_SIZE);
    this->txBusy = false;
}


/***********************************************************************
    Control endpoint
************************************************************************/

void UsbCdc::ep0SendChunk()
{
    uint16_t n = this->ep0Remaining > EP0_SIZE ? EP0_SIZE : this->ep0Remaining;
    HAL_PCD_EP_Transmit(&this->hpcd, 0x80, (uint8_t*)this->ep0Data, n);
    this->ep0Data += n;
    this->ep0Remaining -= n;
}


void UsbCdc::ep0Send(const uint8_t* data, uint16_t len, uint16_t requested)
{
    if (len > requested) len = requested;
    this->ep0Data = data;
    this->ep0Remaining = len;
    this->ep0Zlp = (len < requested) && (len % EP0_SIZE == 0);
    this->ep0InData = true;
    this->ep0SendChunk();
}


void UsbCdc::ep0Status()
{
    this->ep0InData = false;
    HAL_PCD_EP_Transmit(&this->hpcd, 0x80, nullptr, 0);
}


void UsbCdc::ep0Stall()
{
    HAL_PCD_EP_SetStall(&this->hpcd, 0x80);
    HAL_PCD_EP_SetStall(&this->hpcd, 0x00);
}


bool UsbCdc::getDescriptor(uint8_t type, uint8_t index, uint16_t requested)
{
    switch (type)
    {
        case 0x01:
            this->ep0Send(deviceDescriptor, sizeof(deviceDescriptor), requested);
            return true;

        case 0x02:
            this->ep0Send(configDescriptor, sizeof(configDescriptor), requested);
            return true;

        case 0x03:
        {
            if (index == 0)
            {
                this->ep0Send(languageDescriptor, sizeof(languageDescriptor), requested);
                return true;
            }

            const char* text;
            if (index == 1) text = "Remora";
            else if (index == 2) text = "Remora CAN node";
            else if (index == 3) text = this->serial;
            else return false;

            // ASCII to UTF-16LE
            uint8_t n = 0;
            while (text[n] && n < (sizeof(this->stringBuffer) - 2) / 2)
            {
                this->stringBuffer[2 + 2 * n] = (uint8_t)text[n];
                this->stringBuffer[3 + 2 * n] = 0;
                n++;
            }
            this->stringBuffer[0] = (uint8_t)(2 + 2 * n);
            this->stringBuffer[1] = 0x03;
            this->ep0Send(this->stringBuffer, this->stringBuffer[0], requested);
            return true;
        }
    }
    return false;
}


void UsbCdc::onSetup()
{
    const uint8_t* s = (const uint8_t*)this->hpcd.Setup;
    uint8_t  requestType = s[0];
    uint8_t  request = s[1];
    uint16_t value = (uint16_t)(s[2] | (s[3] << 8));
    uint16_t length = (uint16_t)(s[6] | (s[7] << 8));

    this->ep0InData = false;
    this->ep0LineCoding = false;

    if ((requestType & 0x60) == 0x00)                   // standard request
    {
        switch (request)
        {
            case 0x00:                                  // GET_STATUS
                memset(this->ep0Buffer, 0, 2);
                this->ep0Send(this->ep0Buffer, 2, length);
                return;

            case 0x01:                                  // CLEAR_FEATURE
            case 0x03:                                  // SET_FEATURE
            case 0x0B:                                  // SET_INTERFACE
                this->ep0Status();
                return;

            case 0x05:                                  // SET_ADDRESS, applied by the HAL after the status stage
                HAL_PCD_SetAddress(&this->hpcd, (uint8_t)(value & 0x7F));
                this->ep0Status();
                return;

            case 0x06:                                  // GET_DESCRIPTOR
                if (this->getDescriptor((uint8_t)(value >> 8), (uint8_t)value, length)) return;
                break;

            case 0x08:                                  // GET_CONFIGURATION
                this->ep0Buffer[0] = this->configured ? 1 : 0;
                this->ep0Send(this->ep0Buffer, 1, length);
                return;

            case 0x09:                                  // SET_CONFIGURATION
                if (value) this->openEndpoints();
                this->configured = (value != 0);
                this->ep0Status();
                return;

            case 0x0A:                                  // GET_INTERFACE
                this->ep0Buffer[0] = 0;
                this->ep0Send(this->ep0Buffer, 1, length);
                return;
        }
    }
    else if ((requestType & 0x60) == 0x20)              // CDC class request
    {
        switch (request)
        {
            case 0x20:                                  // SET_LINE_CODING, 7 bytes follow
                if (length == 0) break;
                this->ep0LineCoding = true;
                HAL_PCD_EP_Receive(&this->hpcd, 0x00, this->lineCoding, sizeof(this->lineCoding));
                return;

            case 0x21:                                  // GET_LINE_CODING
                this->ep0Send(this->lineCoding, sizeof(this->lineCoding), length);
                return;

            case 0x22:                                  // SET_CONTROL_LINE_STATE
            case 0x23:                                  // SEND_BREAK
                this->ep0Status();
                return;
        }
    }

    this->ep0Stall();
}


void UsbCdc::onDataIn(uint8_t epnum)
{
    if (epnum == 0)
    {
        if (!this->ep0InData) return;                   // status stage done

        if (this->ep0Remaining) this->ep0SendChunk();
        else if (this->ep0Zlp)
        {
            this->ep0Zlp = false;
            HAL_PCD_EP_Transmit(&this->hpcd, 0x80, nullptr, 0);
        }
        else
        {
            this->ep0InData = false;
            HAL_PCD_EP_Receive(&this->hpcd, 0x00, nullptr, 0);      // status stage from the host
        }
    }
    else if (epnum == (EP_DATA_IN & 0x7F))
    {
        this->txBusy = false;
    }
}


void UsbCdc::onDataOut(uint8_t epnum)
{
    if (epnum == 0)
    {
        if (this->ep0LineCoding)
        {
            this->ep0LineCoding = false;
            this->ep0Status();
        }
    }
    else if (epnum == EP_DATA_OUT)
    {
        HAL_PCD_EP_Receive(&this->hpcd, EP_DATA_OUT, this->rxPacket, DATA_SIZE);     // discard
    }
}


void UsbCdc::onReset()
{
    this->configured = false;
    this->txBusy = false;
    this->ep0InData = false;
    this->ep0LineCoding = false;
    HAL_PCD_EP_Open(&this->hpcd, 0x00, EP0_SIZE, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(&this->hpcd, 0x80, EP0_SIZE, EP_TYPE_CTRL);
}


extern "C" void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef* hpcd) { usbCdc.onSetup(); }
extern "C" void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef* hpcd, uint8_t epnum) { usbCdc.onDataIn(epnum); }
extern "C" void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef* hpcd, uint8_t epnum) { usbCdc.onDataOut(epnum); }
extern "C" void HAL_PCD_ResetCallback(PCD_HandleTypeDef* hpcd) { usbCdc.onReset(); }


/***********************************************************************
    stdout (printf) to the USB serial port
************************************************************************/

class UsbConsole : public mbed::FileHandle
{
    public:
        ssize_t write(const void* buffer, size_t size) override { return usbCdc.write(buffer, (int)size); }
        ssize_t read(void* buffer, size_t size) override { return 0; }
        off_t seek(off_t offset, int whence) override { return -ESPIPE; }
        int close() override { return 0; }
        int isatty() override { return 1; }
};

static UsbConsole usbConsole;

mbed::FileHandle* mbed::mbed_override_console(int fd)
{
    return &usbConsole;
}

#endif
