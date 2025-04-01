#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <string.h>
#include "pl455_config.h"
#include "elapsedmillis.h"
#include "module_data.h"
#include "thread_wrapper.h"
#include "gpio.h"

#define SCOPE_SINGLE 0
#define SCOPE_GROUP 1
#define SCOPE_BRDCST 3

#ifndef CONFIG_PL455_LOG_LEVEL
#define CONFIG_PL455_LOG_LEVEL LOG_LEVEL_INF
#endif

class PL455
{
public:
    PL455(GPIO& gpio);

    int wakeup();

    void init();
    uint16_t getModuleVoltage(uint8_t module);
    uint16_t getCellVoltage(uint8_t module, uint8_t cell);
    uint16_t getAuxVoltage(uint8_t module, uint8_t aux);
    int getNumModules();
    uint16_t getMinCellVoltage();
    uint16_t getMaxCellVoltage();
    uint16_t getDifCellVoltage();
    void runBMS();
    bool getBalanceStatus(uint8_t module, uint8_t cell);
    float getTemperature(uint8_t module, uint8_t sensor);
    void fillModuleData(ModuleData& data);


private:
    GPIO& mGPIO;
    const struct device *uartDev;
    struct gpio_dt_spec wakeupGPIO;

    uint16_t CRC16(uint8_t *pBuf, int nLen);
    uint8_t getInitFrame(uint8_t _readWrite, uint8_t scope, uint8_t data_size);
    uint16_t adc2volt(uint16_t adcReading);
    float adc2temp(uint16_t adcReading);
    void send_Frame(uint8_t *message, int messageLength);
    void writeRegister(uint8_t scope, uint8_t device_addr, uint8_t register_addr, const uint8_t *data, uint8_t data_size);
    void readRegister(uint8_t scope, uint8_t device_addr, uint8_t group_id, uint8_t register_addr, uint8_t uint8_tsToReturn);
    void configure();
    void setAddresses();
    void findMinMaxCellVolt();
    void chooseBalanceCells();
    void listenSerial();
    void commReset(bool reset);
    uint8_t numModules = 0;
    bool rxInProgress = 0;
    uint8_t uint8_tsToReceive = 0;
    uint8_t uint8_tsReceived = 0;
    uint8_t registerRequested = 0;
    uint8_t deviceRequested = 0;
    uint8_t scopeRequested = 0;
    uint8_t serialRXbuffer[132];
    bool waitingForResponse = 0;
    bool sentRequest = 0;
    uint16_t moduleVoltages[MAX_MODULES] = {0};    // stores module voltages (raw ADC 16bit values)
    uint16_t cellVoltages[MAX_MODULES][NUM_CELLS] = {0};
    uint16_t auxVoltages[MAX_MODULES][8] = {0};
    uint16_t minCellVoltage = 0;
    uint16_t maxCellVoltage = 0;
    int16_t difCellVoltage = 0;
    bool balanceCells[MAX_MODULES][NUM_CELLS] = {0};
    uint8_t bmsStep = 0;
    unsigned long bmsStepPeriod = 0; // microseconds
    unsigned long bmsStepTime = 0;   // microseconds
    elapsedMillis commTimeout;
    uint8_t bmsSteps;
    uint8_t voltsRequested = 0;

    

    
};