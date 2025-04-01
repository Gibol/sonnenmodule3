#include "slave.h"
#include "can.h"

Slave::Slave(ModuleData &moduleData, uint8_t id, GPIO &gpio) : mData(moduleData), mId(id), mBalancer(gpio), mGPIO(gpio) {}

bool Slave::worker()
{
    mBalancer.runBMS();

    if (lastUpdate > 1000)
    {
        mGPIO.Toggle(GPIO::Name::LED1);

        mBalancer.fillModuleData(mData);

        auto base = BaseAddress + (ModuleOffset * mId);
        CAN_Send(base + ModuleStateOffset, ((uint8_t *)&mData.moduleState), sizeof(ModuleState));

        for (int i = 0; i < 32; i++)
        {
            CAN_Send(base + CellStateOffset + i, ((uint8_t *)&mData.cellStates[i]), sizeof(CellState));
        }

        for (int i = 0; i < 16; i++)
        {
            CAN_Send(base + AdcVoltageOffset + i, ((uint8_t *)&mData.adcStates[i]), sizeof(uint16_t));
        }

        lastUpdate = 0;
        return true;
    }
    return false;
}