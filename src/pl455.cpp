#include "pl455.h"
#include <math.h>

#define BQUART_NODE DT_ALIAS(bquart)
#if !DT_NODE_HAS_STATUS_OKAY(BQUART_NODE)
#error "BOARD does not define bquart"
#endif

#define BQWAKEUP_NODE DT_ALIAS(bqwakeup)
#if !DT_NODE_HAS_STATUS_OKAY(BQWAKEUP_NODE)
#error "BOARD does not define bqwakeup"
#endif

LOG_MODULE_REGISTER(pl455, CONFIG_PL455_LOG_LEVEL);

uint32_t micros()
{
    /* k_cycle_get_32() returns the current hardware cycle count,
       and k_cyc_to_ns_floor32() converts cycles to nanoseconds.
       Dividing by 1000 converts nanoseconds to microseconds. */
    return k_cyc_to_ns_floor32(k_cycle_get_32()) / 1000;
}

static const uint16_t crc16_table[256] = { // CRC16 for PL455 - ITU_T polynomial: x^16 + x^15 + x^2 + 1
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

PL455::PL455(GPIO& gpio) : mGPIO(gpio)
{
    uartDev = DEVICE_DT_GET(BQUART_NODE);
    if (!uartDev)
    {
        LOG_ERR("Error: UART device not found.\n");
        return;
    }

    wakeupGPIO = GPIO_DT_SPEC_GET(BQWAKEUP_NODE, gpios);

    if (!device_is_ready(wakeupGPIO.port))
    {
        LOG_ERR("Error: wakeupGPIO device are not ready\n");
        return;
    }

    gpio_pin_configure_dt(&wakeupGPIO, GPIO_OUTPUT_INACTIVE);

    init();
}

/**
 * Wake up the chip by toggling the wakeup pin.
 *
 * @return 0 on success, negative error code otherwise.
 */
int PL455::wakeup()
{
    int ret = gpio_pin_set(wakeupGPIO.port, wakeupGPIO.pin, 1);
    if (ret < 0)
    {
        LOG_ERR("Error: unable to set wakeup pin high.\n");
        return ret;
    }
    k_sleep(K_MSEC(10)); // Hold high for 10 ms (adjust as needed)
    ret = gpio_pin_set(wakeupGPIO.port, wakeupGPIO.pin, 0);
    if (ret < 0)
    {
        LOG_ERR("Error: unable to set wakeup pin low.\n");
        return ret;
    }
    k_sleep(K_MSEC(10)); // Delay after wakeup pulse
    return 0;
}

// PL455 register settings, stored here as LSuint8_t first
const uint8_t REG03[4] = {0b00000010, 0b11111111, 0b11111111, 0b11111111}; // sample all cells, all aux, and vmodule
const uint8_t REG07[1] = {0b01111011};                                     // sample multiple times on same channel, 12.6us sampling, 8x oversample (recommended by TI)
const uint8_t REG0C[1] = {0b00001000};                                     // start autoaddressing
const uint8_t REG0D[1] = {16};                                             // 16 battery cells
const uint8_t REG0E[1] = {0b00011001};                                     // internal reg enabled, addresses set by autoaddressing, comparators disabled, hysteresis disabled, faults unlatched
const uint8_t REG0F[1] = {0b10000000};                                     // AFE_PCTL enabled (recommended by TI)
const uint8_t REG13[1] = {0b10001000};                                     // balance continues up to 1 second following balancing enable, balancing continues through fault
const uint8_t REG1E[2] = {0b00000001, 0b00000000};                         // enable module voltage readings
const uint8_t REG28[1] = {0x55};                                           // shutdown module 5 seconds after last comm
const uint8_t REG32[1] = {0b00000000};                                     // automonitor off
const uint8_t REG3E[1] = {0xCD};                                           // ADC sample period (60usec  - 0xBB, recommended by TI, but BMW set 0xCD);
const uint8_t REG3F[4] = {0x44, 0x44, 0x44, 0x44};                         // AUX ADC sample period 12.6usec (recommended by TI);

void PL455::commReset(bool reset)
{ 
    // performs comms break or reset
    // pinMode(1, OUTPUT);
    // if (reset == 0)
    // { // comms break
    //     // hold TX low for 12 bit periods
    //     unsigned long lowTime = 12000000 / setBaud;
    //     digitalWrite(1, LOW);
    //     delayMicroseconds(lowTime);
    //     digitalWrite(1, HIGH);
    // }
    // else
    // { // comms reset
    //     // hold TX low for at least 200usec
    //     digitalWrite(1, LOW);
    //     delayMicroseconds(300);
    //     digitalWrite(1, HIGH);
    // }
}

void PL455::findMinMaxCellVolt()
{ 
    // goes through the cell voltages finding the min, max, difference
    minCellVoltage = 65535;
    maxCellVoltage = 0;
    for (uint8_t module = 0; module < numModules; module++)
    {
        for (uint8_t cell = 0; cell < NUM_CELLS; cell++)
        {
            uint16_t cellVolt = cellVoltages[module][cell];
            if (cellVolt > CELL_IGNORE_VOLT)
            { 
                // only process connected cells
                if (cellVolt > maxCellVoltage)
                {
                    maxCellVoltage = cellVolt;
                }
                if (cellVolt < minCellVoltage)
                {
                    minCellVoltage = cellVolt;
                }
            }
        }
    }
    difCellVoltage = maxCellVoltage - minCellVoltage;
    LOG_DBG("min: %dmv\n", minCellVoltage);
    LOG_DBG("max: %dmv\n", maxCellVoltage);
    LOG_DBG("diff: %dmv\n", difCellVoltage);
}

void PL455::chooseBalanceCells()
{ 
    // works out which cells need balancing
    for (int module = 0; module < numModules; module++)
    {
        for (int cell = 0; cell < NUM_CELLS; cell++)
        {
            if ((cellVoltages[module][cell] > minCellVoltage + BALANCE_TOLERANCE) && ((cellVoltages[module][cell] > BALANCE_MIN_VOLT) || ((BALANCE_WHILE_CHARGE == 1) && (1 == 1))))
            { 
                // replace 1==1 with a 'charging' variable
                balanceCells[module][cell] = 1;
            }
            else
            {
                balanceCells[module][cell] = 0;
            }
        }
    }
}

bool PL455::getBalanceStatus(uint8_t module, uint8_t cell)
{ 
    // returns 1 if the cell is balancing
    return balanceCells[module][cell];
}

uint16_t PL455::adc2volt(uint16_t adcReading)
{ 
    // converts ADC readings into voltage, 10ths of a millivolt
    uint32_t voltage = (uint32_t(50000) * uint32_t(adcReading)) / uint32_t(65535);
    return uint16_t(voltage);
}

float PL455::adc2temp(uint16_t adcReading)
{ 
    // converts ADC readings into temperature, degC
    const float To = 290;
    const float Ro = 100000;
    const float Rfix = 150000;
    const float B = 3950;
    float R = (adcReading * Rfix) / (65535 - adcReading);
    float invTemp = (1 / To) + (1 / B) * logf(R / Ro);
    float temperature = (1 / invTemp) - 273;
    return temperature;
}

float PL455::getTemperature(uint8_t module, uint8_t sensor)
{
    return adc2temp(auxVoltages[module][sensor]);
}

uint16_t PL455::getModuleVoltage(uint8_t module)
{ 
    // provides the overall voltage of the chosen module, in hundredths of a volt
    uint16_t moduleADC = moduleVoltages[module];
    uint32_t moduleVolts = (uint32_t(12500) * uint32_t(moduleADC)) / uint32_t(65535);
    return uint16_t(moduleVolts);
}

uint16_t PL455::getCellVoltage(uint8_t module, uint8_t cell)
{ 
    // provides cell voltage, in 10ths of a millivolt
    return adc2volt(cellVoltages[module][cell]);
}

uint16_t PL455::getAuxVoltage(uint8_t module, uint8_t aux)
{ 
    // provides aux input voltage, in 10ths of a millivolt
    return adc2volt(auxVoltages[module][aux]);
}

uint16_t PL455::getMaxCellVoltage()
{
    return adc2volt(maxCellVoltage);
}

uint16_t PL455::getMinCellVoltage()
{
    return adc2volt(minCellVoltage);
}

uint16_t PL455::getDifCellVoltage()
{
    return adc2volt(difCellVoltage);
}

uint8_t PL455::getInitFrame(uint8_t _readWrite, uint8_t scope, uint8_t data_size)
{
    uint8_t initFrame = 0;          // variables for the initialization frame
    uint8_t _data_size = 0;         // bits 2 - 0
    uint8_t _addr_size = ADDR_SIZE; // bit 3. 0 is 8bit address size (recommended by TI), 1 is 16bit (used by BMW)
    uint8_t _req_type = 0;          // bits 6 - 4
    uint8_t _frm_type = 1;          // bit 7. Will always be 1 here (0 is a response back from the PL455s)

    switch (data_size)
    {
    case 0:
        _data_size = 0;
        break;
    case 1:
        _data_size = 1;
        break;
    case 2:
        _data_size = 2;
        break;
    case 3:
        _data_size = 3;
        break;
    case 4:
        _data_size = 4;
        break;
    case 5:
        _data_size = 5;
        break;
    case 6:
        _data_size = 6;
        break;
    case 8:
        _data_size = 7;
        break;
    case 7:
    default: // doesn't accept 7 uint8_ts, or more than 8 uint8_ts of data
        LOG_ERR("ERROR: cannot write with %d bytes!\n", data_size);
        break;
    }

    switch (scope)
    {
    case 0:
    case 1:
    case 3:
        _req_type = _readWrite | (scope << 1);
        break;
    default:
        LOG_ERR("ERROR: Bad init frame scope: %d!\n", scope);
        break;
    }

    initFrame = _data_size | (_addr_size << 3) | (_req_type << 4) | (_frm_type << 7);
    return initFrame;
}

void PL455::writeRegister(uint8_t scope, uint8_t device_addr, uint8_t register_addr, const uint8_t *data, uint8_t data_size)
{
    uint8_t frameSize = 0;
    uint8_t _initFrame = getInitFrame(1, scope, data_size);

    switch (scope)
    {
    case SCOPE_SINGLE:
    case SCOPE_GROUP:
        frameSize = data_size + 3 + ADDR_SIZE; // data, plus register address(es), plus device or group ID, plus command frame (CRC is calculated and added in send_Frame())
        break;
    case SCOPE_BRDCST:
        frameSize = data_size + 2 + ADDR_SIZE; // no group or device ID
        break;
    default: 
        // panic!
        LOG_ERR("ERROR: Bad scope - %d!\n", scope);
    }
    uint8_t frame[frameSize];
    frame[0] = _initFrame;
    switch (scope)
    {
    case SCOPE_SINGLE:
    case SCOPE_GROUP:
        frame[1] = device_addr;
        frame[2] = 0;
        frame[2 + ADDR_SIZE] = register_addr;
        break;
    case SCOPE_BRDCST:
        frame[1] = 0;
        frame[1 + ADDR_SIZE] = register_addr;
        break;
    }
    for (uint8_t i = 0; i < data_size; i++)
    {
        frame[frameSize - data_size + i] = data[data_size - i - 1];
    }
    send_Frame(frame, frameSize);
}

void PL455::readRegister(uint8_t scope, uint8_t device_addr, uint8_t group_id, uint8_t register_addr, uint8_t uint8_tsToReturn)
{
    uint8_t frameSize = 0;
    uint8_t _initFrame = getInitFrame(0, scope, 1);

    switch (scope)
    {
    case SCOPE_SINGLE:
    case SCOPE_BRDCST:
        frameSize = 4 + ADDR_SIZE; // command frame, (max) device address, register address (2 uint8_ts if ADDR_SIZE=1), number of uint8_ts to return
        break;
    case SCOPE_GROUP:
        frameSize = 5 + ADDR_SIZE; // command frame, group ID, register address, max device address, number of uint8_ts to return
        break;
    default: 
        // panic!
        LOG_ERR("ERROR: Bad scope - %d!\n", scope);
    }
    uint8_t frame[frameSize];
    frame[0] = _initFrame;
    switch (scope)
    {
    case SCOPE_GROUP:
        frame[1] = group_id;
        frame[2] = 0;
        frame[2 + ADDR_SIZE] = register_addr;
        frame[3 + ADDR_SIZE] = device_addr;
        break;
    case SCOPE_SINGLE:
        frame[1] = device_addr;
        frame[2] = 0;
        frame[2 + ADDR_SIZE] = register_addr;
        break;
    case SCOPE_BRDCST:
        frame[1] = 0;
        frame[1 + ADDR_SIZE] = register_addr;
        frame[2 + ADDR_SIZE] = device_addr;
    }
    frame[frameSize - 1] = uint8_tsToReturn - 1;
    send_Frame(frame, frameSize);
    registerRequested = register_addr;
    deviceRequested = device_addr;
    scopeRequested = scope;
    waitingForResponse = 1;
    sentRequest = 1;
    commTimeout = 0;
}

void PL455::init()
{   
    //'bmsbaud' sets the baud once running - the first frame is always 250000baud.
    bmsSteps = 100 / (100 - BALANCE_DUTYCYCLE); // managed balancing/voltage measurement
    bmsStepPeriod = BMS_CYCLE_PERIOD / bmsSteps;

    //commReset(1);
    wakeup();
    k_sleep(K_MSEC(100)); 

    uint8_t baudsetting = 1;


    // send packet to change speed to baud
    uint8_t commdata[2] = {0, 0};
    commdata[0] = 0b11100000;       // enable all comms apart from fault
    commdata[1] = baudsetting << 4; // set UART baud
    writeRegister(SCOPE_BRDCST, 0, 0x10, commdata, 2);
    // BMS.flush(); // give time for the serial to be sent

    k_sleep(K_MSEC(2));  // at least 10usec
    
    configure();           // general config
    setAddresses();
    
    // now do per-device configuration
    switch (numModules)
    {
    case 0: // no modules connected!!
        LOG_ERR("ERROR: No CSCs connected!\n");
        break;
    case 1:                       // only one module
        commdata[0] = 0b10000000; // enable UART, disabled all other comms
        writeRegister(SCOPE_SINGLE, 0, 0x10, commdata, 2);
        break;
    default:                      // more than one module
        commdata[0] = 0b11000000; // enable UART and high side comms, not fault - first module
        writeRegister(SCOPE_SINGLE, 0, 0x10, commdata, 2);
        for (int i = 1; i < numModules - 1; i++)
        {
            commdata[0] = 0b01100000; // enable high and low side comms, not fault, disable UART - middle modules
            writeRegister(SCOPE_SINGLE, i, 0x10, commdata, 2);
        }
        commdata[0] = 0b00100000; // enable low side comms, not fault, disable high side and UART - last module
        writeRegister(SCOPE_SINGLE, numModules - 1, 0x10, commdata, 2);
        break;
    }
    commTimeout = 0;
}

void PL455::configure()
{
    writeRegister(SCOPE_BRDCST, 0, 0x07, REG07, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x0D, REG0D, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x0E, REG0E, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x0F, REG0F, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x13, REG13, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x1E, REG1E, 2);
    writeRegister(SCOPE_BRDCST, 0, 0x28, REG28, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x32, REG32, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x03, REG03, 4);
    writeRegister(SCOPE_BRDCST, 0, 0x3E, REG3E, 1);
    writeRegister(SCOPE_BRDCST, 0, 0x3F, REG3F, 4);
}

int PL455::getNumModules()
{
    return int(numModules);
}

void PL455::setAddresses()
{
    writeRegister(SCOPE_BRDCST, 0, 0x0C, REG0C, 1); // starts autoaddressing
    for (uint8_t i = 0; i < MAX_MODULES; i++)
    {
        k_sleep(K_MSEC(20)); 
        uint8_t addr[1] = {i};
        writeRegister(SCOPE_BRDCST, 0, 0x0A, addr, 1); // address 0 - 15
    }
    // all modules will now have an address. Now we check with each one until we get no response.
    elapsedMillis timeout;
    uint8_t checkModule = 0;
    while (timeout < 1000)
    { 
        // allow 1000ms for each module to respond. This should be more than enough!!!
        listenSerial();

        if ((!sentRequest) && (checkModule != MAX_MODULES))
        {                                                        // haven't sent our request yet, but don't ask for addresses > 15 (16th module)
            readRegister(SCOPE_SINGLE, checkModule, 0, 0x0A, 1); // read address of module
            timeout = 0;                                         // don't include time taken to send request
        }
        else if (!waitingForResponse)
        { // just received a response
            if (uint8_tsReceived == 4)
            { // init uint8_t, data uint8_t, 2 CRC uint8_ts
                uint8_t received[uint8_tsReceived];
                for (int i = 0; i < uint8_tsReceived; i++)
                {
                    received[i] = serialRXbuffer[i]; // copy into new array
                }
                if (received[1] == checkModule)
                { // received the right address back
                    checkModule++;
                    timeout = 0;
                    waitingForResponse = 0;
                    sentRequest = 0;
                }
            }
        } // otherwise, we have sent a request, but haven't received a response yet
    }
    // timed out waiting for response, last module must have been the highest address
    numModules = checkModule;
    LOG_INF("Discovered %d modules\n", numModules);
}

uint16_t PL455::CRC16(uint8_t *message, int mlength)
{
    uint16_t CRC = 0;

    for (int i = 0; i < mlength; i++)
    {
        CRC ^= (*message++) & 0x00FF;
        CRC = crc16_table[CRC & 0x00FF] ^ (CRC >> 8);
    }
    return CRC;
}

void PL455::send_Frame(uint8_t *message, int messageLength)
{
    uint16_t CRC;
    CRC = CRC16(message, messageLength);
    uint8_t toSend[messageLength + 2]; //+2 for CRC
    for (int i = 0; i < messageLength; i++)
    {
        toSend[i] = message[i];
    }
    toSend[messageLength] = CRC & 0x00FF;
    toSend[messageLength + 1] = (CRC & 0xFF00) >> 8;
    LOG_DBG("Sending 0x");
    for (int i = 0; i < messageLength + 2; i++)
    {
        uart_poll_out(uartDev, toSend[i]);
        LOG_DBG("%x ", toSend[i]);
    }
    LOG_DBG("\n");
}

void PL455::listenSerial()
{
    uint8_t data;

    // captures all the serial RX
    while (uart_poll_in(uartDev, &data) == 0)
    {
        LOG_DBG("data received: %x\n", data);
        if (waitingForResponse)
        { 
            // if we're not looking for a response, this is definately noise.
            if (!rxInProgress)
            { 
                // first uint8_t of response
                // check this is a response uint8_t
                bool respuint8_t = (data >> 7) & 1;
                uint8_t numuint8_ts = data & 0b01111111;
                if ((respuint8_t == 0) && (numuint8_ts <= 128))
                { 
                    // probably a response uint8_t
                    serialRXbuffer[0] = data;
                    uint8_tsToReceive = numuint8_ts + 1; // number of _data_ uint8_ts to receive. Also receive the init uint8_t, plus two CRC uint8_ts
                    uint8_tsReceived = 1;
                    rxInProgress = 1;
                }
                else
                { 
                    // not a response uint8_t - void
                    rxInProgress = 0;
                    uint8_tsToReceive = 0;
                    uint8_tsReceived = 0;
                }
            }
            else
            { 
                // subsequent uint8_ts
                serialRXbuffer[uint8_tsReceived] = data;
                uint8_tsReceived++;
                LOG_DBG("received: %d, to receive: %d\n", uint8_tsReceived, uint8_tsToReceive);
            }

            if (uint8_tsReceived == uint8_tsToReceive + 3)
            { 
                // received complete frame
                uint8_t completeFrame[uint8_tsReceived];
                LOG_DBG("Received 0x");
                for (int i = 0; i < uint8_tsReceived; i++)
                {
                    completeFrame[i] = serialRXbuffer[i]; // copy frame to new array
                    LOG_DBG("%x ", completeFrame[i]);
                }
                LOG_DBG("\n");
                if (CRC16(completeFrame, uint8_tsReceived) == 0)
                { 
                    // CRC checks ok
                    waitingForResponse = 0;
                    rxInProgress = 0;
                    uint8_tsToReceive = 0;
                    commTimeout = 0;
                }
                else
                { 
                    // void
                    rxInProgress = 0;
                    uint8_tsToReceive = 0;
                    uint8_tsReceived = 0;
                }
            }
        }
    }
}

void PL455::runBMS()
{ 
    mGPIO.Toggle(GPIO::Name::LED0);
    // called frequently from main()
    listenSerial();
    if (commTimeout > COMM_TIMEOUT)
    { 
        // no response
        LOG_ERR("ERROR: comms timeout?\n");
        commTimeout = 500; // don't spam the console
    }
    if (micros() - bmsStepPeriod > bmsStepTime)
    {
        if (bmsStep == 0)
        { 
            // first step - turn off balancing
            uint8_t balanceDisable[2] = {0, 0};
            writeRegister(SCOPE_BRDCST, numModules - 1, 0x14, balanceDisable, 2);
            bmsStepTime = micros();
            bmsStep++;
        }
        else if (bmsStep == 1)
        { 
            // second step, read voltages
            // PL455::readRegister(uint8_t scope, uint8_t device_addr, uint8_t group_id, uint8_t register_addr, uint8_t uint8_tsToReturn);
            if (voltsRequested == 0)
            {                                                                  // nothing requested yet
                readRegister(SCOPE_SINGLE, voltsRequested, 0, 0x02, 1); // slightly abuse the readRegister function to send a command
                voltsRequested++;
            }
            else if (waitingForResponse == 0)
            { 
                // received a response
                // reset flags
                registerRequested = 0;
                deviceRequested = 0;
                scopeRequested = 0;
                sentRequest = 0;
                
                // read response
                for (unsigned int cell = 0; cell < NUM_CELLS; cell++)
                {
                    uint16_t reading;
                    reading = (serialRXbuffer[2 * cell + 1] << 8) | serialRXbuffer[2 * cell + 2];
                    cellVoltages[voltsRequested - 1][15 - cell] = reading;
                }

                for (unsigned int aux = 0; aux < 8; aux++)
                {
                    uint16_t reading;
                    reading = (serialRXbuffer[2 * aux + 33] << 8) | serialRXbuffer[2 * aux + 34];
                    auxVoltages[voltsRequested - 1][7 - aux] = reading;
                }
                
                moduleVoltages[voltsRequested - 1] = (serialRXbuffer[49] << 8) | serialRXbuffer[50];
                
                if (voltsRequested == numModules)
                { 
                    // received the last module data
                    voltsRequested = 0;
                    // update data
                    findMinMaxCellVolt();
                    chooseBalanceCells();
                    // turn balancing back on
                    for (unsigned int module = 0; module < numModules; module++)
                    {
                        uint8_t balanceEnable[2] = {0, 0};
                        for (unsigned int cell = 0; cell < 8; cell++)
                        {
                            balanceEnable[0] = balanceEnable[0] | (balanceCells[module][cell] << cell);
                        }
                        for (unsigned int cell = 8; cell < NUM_CELLS; cell++)
                        {
                            balanceEnable[1] = balanceEnable[1] | (balanceCells[module][cell] << (cell - 8));
                        }
                        writeRegister(SCOPE_SINGLE, module, 0x14, balanceEnable, 2);
                    }
                    // reset flags
                    bmsStepTime = micros();
                    bmsStep++;
                }
                else
                {
                    readRegister(SCOPE_SINGLE, voltsRequested, 0, 0x02, 1);
                    voltsRequested++; // don't reset the step timer - make sure this happens as quick as possible
                }
            }
        }
        else
        { 
            // other steps, keep balancing on
            for (unsigned int module = 0; module < numModules; module++)
            {
                uint8_t balanceEnable[2] = {0, 0};
                for (unsigned int cell = 0; cell < 8; cell++)
                {
                    balanceEnable[0] = balanceEnable[0] | (balanceCells[module][cell] << cell);
                }
                for (unsigned int cell = 8; cell < NUM_CELLS; cell++)
                {
                    balanceEnable[1] = balanceEnable[1] | (balanceCells[module][cell] << (cell - 8));
                }
                writeRegister(SCOPE_SINGLE, module, 0x14, balanceEnable, 2);
            }
            bmsStepTime = micros();
            bmsStep++;
            if (bmsStep >= bmsSteps)
            {
                bmsStep = 0;
            }
        }
    }
}

void PL455::fillModuleData(ModuleData &moduleData)
{
    for (unsigned int module = 0; module < numModules; module++)
    {
        for (unsigned int cell = 0; cell < NUM_CELLS; cell++)
        {
            moduleData.cellStates[module*16 + cell].voltage = getCellVoltage(module, cell);
            moduleData.cellStates[module*16 + cell].balancing = getBalanceStatus(module, cell);
        }
    }
    moduleData.moduleState.m1Voltage = getModuleVoltage(0) / 10;
    moduleData.moduleState.m2Voltage = getModuleVoltage(1) / 10;
    moduleData.moduleState.current = (getAuxVoltage(0, 7) - 25000) * 18;
    moduleData.moduleState.temperature = 250; // 25*C

    for (unsigned int module = 0; module < numModules; module++)
    {
        for (unsigned int adc = 0; adc < 8; adc++)
        {
            moduleData.adcStates[module*8 + adc] = getAuxVoltage(module, adc);
            moduleData.adcStates[module*8 + adc] = getAuxVoltage(module, adc);
        }
    }
}