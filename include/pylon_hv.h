#pragma once

#include <stdint.h>

enum State
{
    Sleep = 0,
    Charge = 1,
    Discharge = 2,
    Idle = 3
} __attribute__((__packed__));

struct StatusBits
{
    State status : 3;
    bool forced_charge_request : 1;
    bool balance_charge_request : 1;
    bool reserved1 : 1;
    bool reserved2 : 1;
    bool reserved3 : 1;
} __attribute__((__packed__));

struct FaultBits
{
    bool voltage_sensor_error : 1;
    bool temperature_sensor_error : 1;
    bool internal_comm_error : 1;
    bool input_over_voltage_error : 1;
    bool input_transposition_error : 1;
    bool relay_check_error : 1;
    bool battery_damage_error : 1;
    bool other_error : 1;
} __attribute__((__packed__));

struct AlarmBits
{
    bool blv : 1;
    bool bhv : 1;
    bool plv : 1;
    bool phv : 1;
    bool clt : 1;
    bool cht : 1;
    bool dlt : 1;
    bool dht : 1;
    bool coca : 1;
    bool doca : 1;
    bool mlv : 1;
    bool mhv : 1;
    bool reserved1 : 1;
    bool reserved2 : 1;
    bool reserved3 : 1;
    bool reserved4 : 1;
} __attribute__((__packed__));

struct ProtectionBits
{
    bool buv : 1;
    bool bov : 1;
    bool puv : 1;
    bool pov : 1;
    bool cut : 1;
    bool cot : 1;
    bool dut : 1;
    bool dot : 1;
    bool coc : 1;
    bool doc : 1;
    bool muv : 1;
    bool mov : 1;
    bool reserved1 : 1;
    bool reserved2 : 1;
    bool reserved3 : 1;
    bool reserved4 : 1;
} __attribute__((__packed__));

struct FaultExtensionBits
{
    bool shutdown_circuit_error : 1;
    bool bmic_error : 1;
    bool internal_bus_error : 1;
    bool power_on_self_test_error : 1;
    bool safety_function_error : 1;
    bool reserved1 : 1;
    bool reserved2 : 1;
    bool reserved3 : 1;
} __attribute__((__packed__));

enum Request : uint8_t
{
    EnsembleInformation = 0,
    SystemEqipmentInformation = 2,
};

namespace Message
{

struct HostRequest
{
    Request request;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
    uint8_t reserved7;
}__attribute__((__packed__));

struct Status // 0x4210
{
    uint16_t total_voltage;
    uint16_t current;
    uint16_t temperature;
    uint8_t soc;
    uint8_t soh;
} __attribute__((__packed__));

struct ChargeDischargeParameters //0x4220
{
    uint16_t charge_cutoff_voltage;
    uint16_t discharge_cutoff_voltage;
    uint16_t max_charge_current;
    uint16_t max_discharge_current;
} __attribute__((__packed__));

struct CellVoltageStatus //0x4230
{
    uint16_t max_cell_voltage;
    uint16_t min_cell_voltage;
    uint16_t max_cell_voltage_index;
    uint16_t min_cell_voltage_index;
} __attribute__((__packed__));

struct CellTemperatureStatus //0x4240
{
    uint16_t max_cell_temp;
    uint16_t min_cell_temp;
    uint16_t max_temp_cell_index;
    uint16_t min_temp_cell_index;
} __attribute__((__packed__));

struct Bits //0x4250
{
    StatusBits basic_status;
    uint16_t cycle_period;
    FaultBits error;
    AlarmBits alarm;
    ProtectionBits protection;
} __attribute__((__packed__));

struct ModuleVoltageStatus //0x4260
{
    uint16_t module_max_voltage;
    uint16_t module_min_voltage;
    uint16_t module_max_voltage_index;
    uint16_t module_min_voltage_index;
} __attribute__((__packed__));

struct ModuleTemperatureStatus //0x4270
{
    uint16_t module_max_temp;
    uint16_t module_min_temp;
    uint16_t module_max_temp_index;
    uint16_t module_min_temp_index;
} __attribute__((__packed__));

struct ChargeDischargeStatus //0x4280
{
    uint8_t charge_forbidden;
    uint8_t discharge_forbidden;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
    uint8_t reserved7;
} __attribute__((__packed__));

struct FaultExtensionInfo //0x4090
{
    FaultExtensionBits fault_ext1;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
    uint8_t reserved7;
} __attribute__((__packed__));

//0x42E0 - unknown FF 00 00 00 00 00 00 00
//0x42F0 - DynessHV :) GibolHV!
//0x4300 - unknown 03 00 00 00 00 00 00 00

struct SystemEquipmentInfo1 //0x7310
{
    uint8_t hardware_version_code;
    uint8_t reserved;
    uint8_t hardware_version_v;
    uint8_t hardware_version_r;
    uint8_t software_version_major;
    uint8_t software_version_minor;
    uint8_t software_dev_major;
    uint8_t software_dev_minor;
}__attribute__((__packed__)) ;

struct SystemEquipmentInfo2 //0x7320
{
    uint16_t battery_module_qty;
    uint8_t battery_modules_in_series;
    uint8_t cell_qty_per_module;
    uint16_t voltage_level;
    uint16_t ah_number;
}__attribute__((__packed__));

}

constexpr uint32_t CAN_ID_STATUS                    = 0x4210; 
constexpr uint32_t CAN_ID_CHARGE_DISCHARGE_PARAMS   = 0x4220; 
constexpr uint32_t CAN_ID_CELL_VOLTAGE_STATUS       = 0x4230;
constexpr uint32_t CAN_ID_CELL_TEMPERATURE_STATUS   = 0x4240; 
constexpr uint32_t CAN_ID_BITS                      = 0x4250; 
constexpr uint32_t CAN_ID_MODULE_VOLTAGE_STATUS     = 0x4260; 
constexpr uint32_t CAN_ID_MODULE_TEMPERATURE_STATUS = 0x4270; 
constexpr uint32_t CAN_ID_CHARGE_DISCHARGE_STATUS   = 0x4280; 
constexpr uint32_t CAN_ID_FAULT_EXTENSION_INFO      = 0x4290; 
