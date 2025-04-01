#pragma once

#include "module_data.h"
#include "pl455.h"
#include "gpio.h"
#include "elapsedmillis.h"

class Slave
{
    public:
    Slave(ModuleData& moduleData, uint8_t id, GPIO &gpio);
    bool worker();

    private:
    uint8_t mId;
    ModuleData &mData;
    PL455 mBalancer;
    GPIO& mGPIO;
    elapsedMillis lastUpdate;
};