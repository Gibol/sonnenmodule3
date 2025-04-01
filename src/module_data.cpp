#include "module_data.h"
#include <zephyr/kernel.h>

void ModuleData::SetRawData(uint32_t address, uint8_t *data)
{
    if(isComplete())
    {
        cellStatesUpdateFlags = 0;
        adcUpdateFlags = 0;
        moduleStateFlag = false;
    }

    if((address & DataTypeMask) == ModuleStateOffset)
    {
        moduleState = *reinterpret_cast<ModuleState *>(data);
        moduleStateFlag = true;
    }
    else if((address & DataTypeMask) == CellStateOffset)
    {
        uint32_t channel = address & DataChannelMask;
        if(channel < 32)
        {
            cellStates[channel] = *reinterpret_cast<CellState *>(data);
            cellStatesUpdateFlags |= (1 << channel);
        }
    }
    else if((address & DataTypeMask) == AdcVoltageOffset)
    {
        uint32_t channel = address & DataChannelMask;
        if(channel < 16)
        {
            adcStates[channel] = *reinterpret_cast<uint16_t *>(data);
            adcUpdateFlags |= (1 << channel);
        }
    }
}

bool ModuleData::isComplete()
{
    return moduleStateFlag && (cellStatesUpdateFlags == 0xFFFFFFFF) && (adcUpdateFlags == 0xFFFF);
}