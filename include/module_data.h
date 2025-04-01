#pragma once

#include <stdint.h>

constexpr uint32_t BaseAddress = 	0x11DD0000;
constexpr uint32_t ModuleOffset = 		0x1000;
constexpr uint32_t ModuleStateOffset = 	 0x000;
constexpr uint32_t CellStateOffset = 	 0x100;
constexpr uint32_t AdcVoltageOffset =    0x200;
constexpr uint32_t DataTypeMask =   	 0xF00;
constexpr uint32_t DataChannelMask =   	  0xFF;
constexpr uint32_t IdMask =   		 	0xF000;

struct CellState
{
	uint16_t voltage; //in 0,1mV steps
	uint8_t balancing; // indicates if cell is currently balanced
} __attribute__((packed));

struct ModuleState
{
	// m1 + m2 gives module voltage, m1 is for first 16 cells m2 is for second 16 cells
	uint16_t m1Voltage; //in 0,1V steps
	uint16_t m2Voltage; //in 0,1V steps
	int16_t current; //in 0,1mA steps
	uint16_t temperature; //in 0,1C steps
} __attribute__((packed));

struct ModuleData
{
    ModuleState moduleState;
    CellState cellStates[32];
	uint16_t adcStates[16];
	void SetRawData(uint32_t address, uint8_t *data);
	uint32_t cellStatesUpdateFlags = 0;
	uint16_t adcUpdateFlags = 0;
	bool moduleStateFlag = false;
	bool isComplete();
};