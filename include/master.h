#pragma once

#include "pylon_hv.h"
#include "module_data.h"
#include <cstddef>
#include "gpio.h"

#ifndef CONFIG_MASTER_BMS_LOG_LEVEL
#define CONFIG_MASTER_BMS_LOG_LEVEL LOG_LEVEL_INF
#endif

#define MODULE_DATA_TIMEOUT_MS 5000

#define NUM_MODULES 2

class MasterBMS {
public:
    MasterBMS(GPIO& gpio);

    // Update data for a specific module
    // Returns true if successful, false on invalid index
    bool updateModuleData(uint8_t moduleIndex, const ModuleData& data);

    // Process all received module data to update master status
    void processData();

    // --- Getters for Output Data ---
    const Message::Status& getStatus() const;
    const Message::ChargeDischargeParameters& getChargeDischargeParameters() const;
    const Message::CellVoltageStatus& getCellVoltageStatus() const;
    const Message::CellTemperatureStatus& getCellTemperatureStatus() const;
    const Message::Bits& getBits() const;
    const Message::ModuleVoltageStatus& getModuleVoltageStatus() const;
    const Message::ModuleTemperatureStatus& getModuleTemperatureStatus() const;
    const Message::ChargeDischargeStatus& getChargeDischargeStatus() const;
    const Message::FaultExtensionInfo& getFaultExtensionInfo() const;

    // --- Host Request Handling (Example) ---
    void handleHostRequest(Request request);
    void worker(k_msgq& queue);


private:
    // Compile-time check for number of modules
    static_assert(NUM_MODULES > 0, "Number of modules must be greater than zero.");

    GPIO& mGPIO;

    // Internal Data Storage (C-Style Arrays)
    ModuleData moduleData_[NUM_MODULES] = {};
    bool initializedModules_[NUM_MODULES] = {0}; // Track if initial data received
    int64_t lastUpdateTimeMs_[NUM_MODULES] = {0}; // Track data freshness
    bool allModulesInitialized_ = false;
    bool communicationOk_ = false; // Tracks if all modules are communicating within timeout

    // Output data storage
    Message::Status outputStatus_{};
    Message::ChargeDischargeParameters outputChargeDischargeParams_{};
    Message::CellVoltageStatus outputCellVoltageStatus_{};
    Message::CellTemperatureStatus outputCellTemperatureStatus_{};
    Message::Bits outputBits_{};
    Message::ModuleVoltageStatus outputModuleVoltageStatus_{};
    Message::ModuleTemperatureStatus outputModuleTemperatureStatus_{};
    Message::ChargeDischargeStatus outputChargeDischargeStatus_{};
    Message::FaultExtensionInfo outputFaultExt_{};

    // --- Placeholder Thresholds (DEFINE THESE BASED ON LFP DATASHEET AND SYSTEM REQUIREMENTS) ---
    // Cell Voltages in 0.1mV
    static constexpr uint16_t CELL_OVER_VOLTAGE_PROTECTION_THRESHOLD_01MV = 36500; // 3.65V
    static constexpr uint16_t CELL_UNDER_VOLTAGE_PROTECTION_THRESHOLD_01MV = 25000; // 2.50V
    static constexpr uint16_t CELL_OVER_VOLTAGE_ALARM_THRESHOLD_01MV = 36000;    // 3.60V
    static constexpr uint16_t CELL_UNDER_VOLTAGE_ALARM_THRESHOLD_01MV = 27000;   // 2.70V

    // Module Voltages in 0.1V (Sum of m1+m2)
    static constexpr uint16_t MODULE_OVER_VOLTAGE_PROTECTION_THRESHOLD_01V = (365 * 32) / 10; // 3.65V/cell * 32 cells -> 116.8V -> 1168 * 0.1V
    static constexpr uint16_t MODULE_UNDER_VOLTAGE_PROTECTION_THRESHOLD_01V = (250 * 32) / 10; // 2.50V/cell * 32 cells -> 80.0V -> 800 * 0.1V
    static constexpr uint16_t MODULE_OVER_VOLTAGE_ALARM_THRESHOLD_01V = (360 * 32) / 10;     // 3.60V/cell * 32 cells -> 115.2V -> 1152 * 0.1V
    static constexpr uint16_t MODULE_UNDER_VOLTAGE_ALARM_THRESHOLD_01V = (270 * 32) / 10;    // 2.70V/cell * 32 cells -> 86.4V -> 864 * 0.1V

    // Temperatures in 0.1C (Non-negative)
    static constexpr uint16_t CHARGE_OVER_TEMP_PROTECTION_THRESHOLD_01C = 500;   // 50.0 C
    static constexpr uint16_t CHARGE_UNDER_TEMP_PROTECTION_THRESHOLD_01C = 0;      // 0.0 C
    static constexpr uint16_t DISCHARGE_OVER_TEMP_PROTECTION_THRESHOLD_01C = 600;  // 60.0 C
    static constexpr uint16_t DISCHARGE_UNDER_TEMP_PROTECTION_THRESHOLD_01C = 10; // 1.0 C (Example, avoid discharge below freezing if needed, was -200 before)
    static constexpr uint16_t CHARGE_HIGH_TEMP_ALARM_THRESHOLD_01C = 450;    // 45.0 C
    static constexpr uint16_t CHARGE_LOW_TEMP_ALARM_THRESHOLD_01C = 50;     // 5.0 C
    static constexpr uint16_t DISCHARGE_HIGH_TEMP_ALARM_THRESHOLD_01C = 550;   // 55.0 C
    static constexpr uint16_t DISCHARGE_LOW_TEMP_ALARM_THRESHOLD_01C = 50;    // 5.0 C (Was -100 before)

    // Currents in 0.1A (System Level)
    // Note: Input current is 0.1mA, requires scaling
    static constexpr int16_t CHARGE_OVER_CURRENT_PROTECTION_THRESHOLD_01A = 190; // 19A Charge
    static constexpr int16_t DISCHARGE_OVER_CURRENT_PROTECTION_THRESHOLD_01A = -190;// 19A Discharge (Negative)
    static constexpr int16_t CHARGE_OVER_CURRENT_ALARM_THRESHOLD_01A = 180;  // 18A Charge
    static constexpr int16_t DISCHARGE_OVER_CURRENT_ALARM_THRESHOLD_01A = -180; // 18A Discharge (Negative)

    // SOC Derating Thresholds (%)
    static constexpr uint8_t SOC_HIGH_CHARGE_DERATE_START = 90;         // Start reducing charge current above this SOC
    static constexpr uint8_t SOC_NEAR_FULL_CHARGE_DERATE_START = 95;    // Reduce charge current further
    static constexpr uint8_t SOC_LOW_DISCHARGE_DERATE_START = 10;       // Start reducing discharge current below this SOC
    static constexpr uint8_t SOC_NEAR_EMPTY_DISCHARGE_DERATE_START = 5; // Reduce discharge current further

    // Cell Imbalance Threshold (0.1mV) - Reduce current if max-min cell voltage exceeds this
    static constexpr uint16_t CELL_IMBALANCE_DERATE_THRESHOLD_01MV = 1000; // 100mV example

    // SOH Derating Thresholds (%) - Reduce current below these SOH values
    static constexpr uint8_t SOH_DERATE_LEVEL1_THRESHOLD = 90;
    static constexpr uint8_t SOH_DERATE_LEVEL2_THRESHOLD = 80;
    static constexpr uint8_t SOH_DERATE_LEVEL3_THRESHOLD = 70;

    // Derating Factors (0.0 to 1.0) - Multipliers applied to base current limit
    static constexpr float TEMP_DERATE_FACTOR = 0.5f;
    static constexpr float SOC_HIGH_CHARGE_FACTOR = 0.5f;
    static constexpr float SOC_NEAR_FULL_CHARGE_FACTOR = 0.2f;
    static constexpr float SOC_LOW_DISCHARGE_FACTOR = 0.5f;
    static constexpr float SOC_NEAR_EMPTY_DISCHARGE_FACTOR = 0.2f;
    static constexpr float IMBALANCE_DERATE_FACTOR = 0.5f;
    static constexpr float SOH_DERATE_LEVEL1_FACTOR = 0.9f;
    static constexpr float SOH_DERATE_LEVEL2_FACTOR = 0.8f;
    static constexpr float SOH_DERATE_LEVEL3_FACTOR = 0.7f;

    // System Voltage Limits in 0.1V (Example values)
    // Calculated based on NUM_MODULES, defined in constructor or method
    uint16_t SYSTEM_CHARGE_CUTOFF_VOLTAGE_01V = (360 * 32 * NUM_MODULES) / 10; // ~3.6V per cell
    uint16_t SYSTEM_DISCHARGE_CUTOFF_VOLTAGE_01V = (280 * 32 * NUM_MODULES) / 10; // ~2.8V per cell


    // --- Private Helper Methods ---
    void resetOutputs();
    void checkAllModulesInitialized();
    bool checkCommunicationTimeout(); // Returns true if communication is OK

    // Calculation Helpers
    uint8_t calculateSOC(uint16_t min_cell_voltage);
    uint8_t calculateSOH();
    uint16_t calculateMaxChargeCurrent(); // Returns absolute value in 0.1A
    uint16_t calculateMaxDischargeCurrent(); // Returns absolute value in 0.1A
    State determineSystemState(int16_t);
};
