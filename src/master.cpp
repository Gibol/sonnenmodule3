#include "master.h"
#include <zephyr/logging/log.h>
#include "can.h"
#include <limits.h>
#include <stdlib.h>
#include <math.h>

// Register Zephyr log module
LOG_MODULE_REGISTER(master_bms, CONFIG_MASTER_BMS_LOG_LEVEL); // Use Kconfig level

// --- SOC Look-Up Table (Voltage-based, Simple LFP approximation) ---
// Place in anonymous namespace or make static const
namespace {
    struct SocPoint {
        uint16_t voltage_01mV; // Cell voltage in 0.1mV
        uint8_t soc_percent;  // Corresponding SOC %
    };

    // Example LUT - **MUST BE ADJUSTED BASED ON SPECIFIC LFP CELL DATASHEET**
    const SocPoint socLut[] = {
        { 25000, 0 },    // 2.50V - Protection Cutoff
        { 28000, 5 },    // 2.80V - Near Empty
        { 31000, 10 },   // 3.10V - Entering flat zone
        { 32000, 20 },   // 3.20V - Flat zone
        { 32500, 40 },   // 3.25V - Flat zone
        { 33000, 80 },   // 3.30V - Flat zone
        { 33500, 95 },   // 3.35V - Leaving flat zone
        { 34500, 98 },   // 3.45V - Near Full
        { 36000, 100 },  // 3.60V - Alarm/Full
        { 36500, 100 }   // 3.65V - Protection Cutoff
    };
    const size_t socLutSize = sizeof(socLut) / sizeof(socLut[0]);

    // Helper for linear interpolation
    uint8_t interpolate(uint16_t currentVoltage, const SocPoint& p1, const SocPoint& p2) {
        if (p2.voltage_01mV == p1.voltage_01mV) {
            // Avoid division by zero, return average or one end
            return p1.soc_percent;
        }
        // Ensure we don't calculate negative SOC for uint8_t due to voltage order issues if any
        if (currentVoltage <= p1.voltage_01mV) return p1.soc_percent;
        if (currentVoltage >= p2.voltage_01mV) return p2.soc_percent;

        // Linear interpolation: soc = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        // Use larger types for intermediate calculation to avoid overflow/precision loss
        int32_t voltage_diff = p2.voltage_01mV - p1.voltage_01mV;
        int32_t soc_diff = p2.soc_percent - p1.soc_percent;
        int32_t voltage_offset = currentVoltage - p1.voltage_01mV;

        // Perform calculation carefully
        int32_t interpolated_soc = p1.soc_percent + (voltage_offset * soc_diff) / voltage_diff;

        // Clamp results just in case
        if (interpolated_soc < 0) return 0;
        if (interpolated_soc > 100) return 100;

        return static_cast<uint8_t>(interpolated_soc);
    }

} // anonymous namespace


// --- Constructor ---

MasterBMS::MasterBMS(GPIO &gpio) :
    mGPIO(gpio),
    allModulesInitialized_(false),
    communicationOk_(false) // Assume not OK until first check passes
{
    int64_t now = k_uptime_get();
    for (size_t i = 0; i < NUM_MODULES; ++i) {
        lastUpdateTimeMs_[i] = now; // Initialize to current time to avoid immediate timeout
        // moduleData_[i] is default initialized
    }
    resetOutputs();

    // Calculate dynamic system voltage limits based on NUM_MODULES
    // Ensure calculations don't overflow uint16_t if NUM_MODULES is very large
    SYSTEM_CHARGE_CUTOFF_VOLTAGE_01V = static_cast<uint16_t>((360UL * 32 * NUM_MODULES) / 10);
    SYSTEM_DISCHARGE_CUTOFF_VOLTAGE_01V = static_cast<uint16_t>((280UL * 32 * NUM_MODULES) / 10);

    LOG_INF("MasterBMS initialized for %u modules.", NUM_MODULES);
}

// --- Update Module Data ---

bool MasterBMS::updateModuleData(uint8_t moduleIndex, const ModuleData& data) 
{
    if (moduleIndex >= NUM_MODULES) {
        LOG_WRN("Invalid module index %u received.", moduleIndex);
        return false;
    }
    moduleData_[moduleIndex] = data;
    lastUpdateTimeMs_[moduleIndex] = k_uptime_get(); // Record update time

    if (!initializedModules_[moduleIndex]) {
        initializedModules_[moduleIndex] = true;
        checkAllModulesInitialized(); // Check if all modules reported in
    }
    LOG_INF("Updated data for module %u", moduleIndex);
    return true;
}

// --- Process Data ---

void MasterBMS::processData() {

    // 1. Check Communication Status
    communicationOk_ = checkCommunicationTimeout();
    if (!allModulesInitialized_ || !communicationOk_) {
        outputBits_.error.internal_comm_error = true;
        // Keep outputs in a safe state (e.g., Idle, charge/discharge forbidden)
        // Reset may clear previous state, so set forbidden flags explicitly
        outputChargeDischargeStatus_.charge_forbidden = 1;
        outputChargeDischargeStatus_.discharge_forbidden = 1;
        outputBits_.basic_status.status = State::Idle;
        LOG_WRN("Processing skipped: Communication timeout or not all modules initialized.");
        return; // Cannot process reliably
    }

     // --- Reset temporary aggregators and flags (only if communication is OK) ---
    uint32_t totalVoltage_01V = 0; // Use 0.1V units based on ModuleState input
    uint16_t minCellVoltage_01mV = USHRT_MAX;
    uint16_t maxCellVoltage_01mV = 0; // Min is 0
    uint16_t minCellIndex = 0;
    uint16_t maxCellIndex = 0;

    uint16_t minModuleVoltage_01V = USHRT_MAX;
    uint16_t maxModuleVoltage_01V = 0;
    uint8_t minModuleVoltageIndex = 0;
    uint8_t maxModuleVoltageIndex = 0;

    uint16_t minModuleTemp_01C = USHRT_MAX;// Non-negative temp
    uint16_t maxModuleTemp_01C = 0; // Non-negative temp
    uint8_t minModuleTempIndex = 0;
    uint8_t maxModuleTempIndex = 0;

    int32_t totalCurrent_01mA = 0; // Accumulate current for averaging or checking consistency

    // Reset flags (assume OK until proven otherwise)
    outputBits_.error = {}; // Clear previous errors (except comm error handled above)
    outputBits_.alarm = {};
    outputBits_.protection = {};
    outputFaultExt_.fault_ext1 = {};
    outputChargeDischargeStatus_.charge_forbidden = 0;
    outputChargeDischargeStatus_.discharge_forbidden = 0;


    // --- Iterate through modules ---
    for (size_t i = 0; i < NUM_MODULES; ++i) {
        const auto& modData = moduleData_[i];
        const auto& modState = modData.moduleState;

        // Module Voltage Calculation (Unit: 0.1V)
        uint16_t currentModuleVoltage_01V = modState.m1Voltage + modState.m2Voltage;
        totalVoltage_01V += currentModuleVoltage_01V;

        if (modState.m1Voltage < minModuleVoltage_01V) {
            minModuleVoltage_01V = modState.m1Voltage;
            minModuleVoltageIndex = static_cast<uint8_t>(i*2);
        }
        if (modState.m1Voltage > maxModuleVoltage_01V) {
            maxModuleVoltage_01V = modState.m1Voltage;
            maxModuleVoltageIndex = static_cast<uint8_t>(i*2);
        }

        if (modState.m2Voltage < minModuleVoltage_01V) {
            minModuleVoltage_01V = modState.m2Voltage;
            minModuleVoltageIndex = static_cast<uint8_t>((i*2)+1);
        }
        if (modState.m2Voltage > maxModuleVoltage_01V) {
            maxModuleVoltage_01V = modState.m2Voltage;
            maxModuleVoltageIndex = static_cast<uint8_t>((i*2)+1);
        }

        // Module Temperature (Unit: 0.1C, uint16_t)
        uint16_t currentModuleTemp_01C = modState.temperature;
        if (currentModuleTemp_01C < minModuleTemp_01C) {
            minModuleTemp_01C = currentModuleTemp_01C;
            minModuleTempIndex = static_cast<uint8_t>(i);
        }
        if (currentModuleTemp_01C > maxModuleTemp_01C) {
            maxModuleTemp_01C = currentModuleTemp_01C;
            maxModuleTempIndex = static_cast<uint8_t>(i);
        }

        // Accumulate Current (Unit: 0.1mA, int16_t)
        // For series strings, current should be similar. Averaging might smooth noise.
        // For parallel strings, need to sum. Assuming series for now, averaging.
        totalCurrent_01mA += modState.current;

        // Cell Voltages within the module (Unit: 0.1mV)
        for (size_t j = 0; j < 32; ++j) {
            const auto& cellState = modData.cellStates[j];
            uint16_t currentCellVoltage_01mV = cellState.voltage;
            uint16_t absoluteCellIndex = static_cast<uint16_t>(i * 32 + j);

            if (currentCellVoltage_01mV < minCellVoltage_01mV) {
                minCellVoltage_01mV = currentCellVoltage_01mV;
                minCellIndex = absoluteCellIndex;
            }
            if (currentCellVoltage_01mV > maxCellVoltage_01mV) {
                maxCellVoltage_01mV = currentCellVoltage_01mV;
                maxCellIndex = absoluteCellIndex;
            }

            // --- Check Individual Cell Alarms/Protections ---
             if (currentCellVoltage_01mV > CELL_OVER_VOLTAGE_PROTECTION_THRESHOLD_01MV) {
                 outputBits_.protection.pov = true; // Pack Over Voltage (cell level)
                 outputBits_.protection.bov = true; // Battery Over Voltage (system level)
                 outputChargeDischargeStatus_.charge_forbidden = 1;
             }
             if (currentCellVoltage_01mV < CELL_UNDER_VOLTAGE_PROTECTION_THRESHOLD_01MV) {
                 outputBits_.protection.puv = true; // Pack Under Voltage (cell level)
                 outputBits_.protection.buv = true; // Battery Under Voltage (system level)
                 outputChargeDischargeStatus_.discharge_forbidden = 1;
             }
             if (currentCellVoltage_01mV > CELL_OVER_VOLTAGE_ALARM_THRESHOLD_01MV) {
                 outputBits_.alarm.phv = true; // Pack High Voltage Alarm
                 outputBits_.alarm.bhv = true; // Battery High Voltage Alarm
             }
              if (currentCellVoltage_01mV < CELL_UNDER_VOLTAGE_ALARM_THRESHOLD_01MV) {
                 outputBits_.alarm.plv = true; // Pack Low Voltage Alarm
                 outputBits_.alarm.blv = true; // Battery Low Voltage Alarm
             }
        } // End cell loop

         // --- Check Module Voltage Alarms/Protections (Unit: 0.1V) ---
         if (currentModuleVoltage_01V > MODULE_OVER_VOLTAGE_PROTECTION_THRESHOLD_01V) {
             outputBits_.protection.mov = true;
             outputChargeDischargeStatus_.charge_forbidden = 1;
         }
          if (currentModuleVoltage_01V < MODULE_UNDER_VOLTAGE_PROTECTION_THRESHOLD_01V) {
             outputBits_.protection.muv = true;
             outputChargeDischargeStatus_.discharge_forbidden = 1;
         }
          if (currentModuleVoltage_01V > MODULE_OVER_VOLTAGE_ALARM_THRESHOLD_01V) {
             outputBits_.alarm.mhv = true;
         }
          if (currentModuleVoltage_01V < MODULE_UNDER_VOLTAGE_ALARM_THRESHOLD_01V) {
             outputBits_.alarm.mlv = true;
         }

        // --- Check Module Temperature Alarms/Protections (Unit: 0.1C) ---
         if (currentModuleTemp_01C > CHARGE_OVER_TEMP_PROTECTION_THRESHOLD_01C) {
             outputBits_.protection.cot = true;
             outputChargeDischargeStatus_.charge_forbidden = 1;
         }
         if (currentModuleTemp_01C < CHARGE_UNDER_TEMP_PROTECTION_THRESHOLD_01C) {
             outputBits_.protection.cut = true;
              outputChargeDischargeStatus_.charge_forbidden = 1;
         }
        if (currentModuleTemp_01C > DISCHARGE_OVER_TEMP_PROTECTION_THRESHOLD_01C) {
             outputBits_.protection.dot = true;
             outputChargeDischargeStatus_.discharge_forbidden = 1;
         }
         if (currentModuleTemp_01C < DISCHARGE_UNDER_TEMP_PROTECTION_THRESHOLD_01C) {
             outputBits_.protection.dut = true;
             outputChargeDischargeStatus_.discharge_forbidden = 1;
         }
         // Alarms
         if (currentModuleTemp_01C > CHARGE_HIGH_TEMP_ALARM_THRESHOLD_01C) outputBits_.alarm.cht = true;
         if (currentModuleTemp_01C < CHARGE_LOW_TEMP_ALARM_THRESHOLD_01C) outputBits_.alarm.clt = true;
         if (currentModuleTemp_01C > DISCHARGE_HIGH_TEMP_ALARM_THRESHOLD_01C) outputBits_.alarm.dht = true;
         if (currentModuleTemp_01C < DISCHARGE_LOW_TEMP_ALARM_THRESHOLD_01C) outputBits_.alarm.dlt = true;

         // --- Placeholder: Check for other module-specific errors reported by module ---
         // Example: if (modState.some_internal_error_flag) outputBits_.error.other_error = true;

    } // End of module loop


    // --- Aggregate and Finalize Outputs ---

    // Status Message (0x4210)
    outputStatus_.total_voltage = static_cast<uint16_t>(totalVoltage_01V); // Already in 0.1V units but needs another division???
    // Calculate average current (0.1mA), then convert to 0.1A
    int32_t avgCurrent_01mA = totalCurrent_01mA / NUM_MODULES;
    outputStatus_.current = static_cast<uint16_t>((avgCurrent_01mA / 1000) + 30000); // Convert 0.1mA to 0.1A offset -3000A
    // Use max module temp for overall temp (or average)
    outputStatus_.temperature = maxModuleTemp_01C + 1000; // Already in 0.1C units offset -100c
    outputStatus_.soc = calculateSOC(minCellVoltage_01mV);
    outputStatus_.soh = calculateSOH();

    // Charge/Discharge Parameters (0x4220)
    outputChargeDischargeParams_.charge_cutoff_voltage = SYSTEM_CHARGE_CUTOFF_VOLTAGE_01V; // Pre-calculated
    outputChargeDischargeParams_.discharge_cutoff_voltage = SYSTEM_DISCHARGE_CUTOFF_VOLTAGE_01V; // Pre-calculated
    outputChargeDischargeParams_.max_charge_current = calculateMaxChargeCurrent() + 30000;
    outputChargeDischargeParams_.max_discharge_current = -calculateMaxDischargeCurrent() + 30000;

    // Cell Voltage Status (0x4230)
    outputCellVoltageStatus_.max_cell_voltage = maxCellVoltage_01mV / 10;
    outputCellVoltageStatus_.min_cell_voltage = minCellVoltage_01mV / 10;
    outputCellVoltageStatus_.max_cell_voltage_index = maxCellIndex;
    outputCellVoltageStatus_.min_cell_voltage_index = minCellIndex;

    // Cell Temperature Status (0x4240) - Using Module Temps as discussed
    outputCellTemperatureStatus_.max_cell_temp = maxModuleTemp_01C + 1000;
    outputCellTemperatureStatus_.min_cell_temp = minModuleTemp_01C + 1000;
    outputCellTemperatureStatus_.max_temp_cell_index = maxModuleTempIndex; // Reporting module index
    outputCellTemperatureStatus_.min_temp_cell_index = minModuleTempIndex; // Reporting module index

    // Module Voltage Status (0x4260)
    outputModuleVoltageStatus_.module_max_voltage = maxModuleVoltage_01V * 100; // Already in 0.1V
    outputModuleVoltageStatus_.module_min_voltage = minModuleVoltage_01V * 100; // Already in 0.1V
    outputModuleVoltageStatus_.module_max_voltage_index = maxModuleVoltageIndex;
    outputModuleVoltageStatus_.module_min_voltage_index = minModuleVoltageIndex;

    // Module Temperature Status (0x4270)
    outputModuleTemperatureStatus_.module_max_temp = maxModuleTemp_01C + 1000; // Already in 0.1C
    outputModuleTemperatureStatus_.module_min_temp = minModuleTemp_01C + 1000; // Already in 0.1C
    outputModuleTemperatureStatus_.module_max_temp_index = maxModuleTempIndex;
    outputModuleTemperatureStatus_.module_min_temp_index = minModuleTempIndex;

    // --- System Current Alarms/Protections ---
    int16_t system_current_01A = avgCurrent_01mA / 1000; // Use calculated system current (0.1A)
    if (system_current_01A > CHARGE_OVER_CURRENT_PROTECTION_THRESHOLD_01A) {
         outputBits_.protection.coc = true;
         outputChargeDischargeStatus_.charge_forbidden = 1;
    }
    // Check discharge current (negative value)
    if (system_current_01A < DISCHARGE_OVER_CURRENT_PROTECTION_THRESHOLD_01A) {
         outputBits_.protection.doc = true;
         outputChargeDischargeStatus_.discharge_forbidden = 1;
     }
     if (system_current_01A > CHARGE_OVER_CURRENT_ALARM_THRESHOLD_01A) {
        outputBits_.alarm.coca = true;
     }
     // Check discharge current alarm (negative value)
     if (system_current_01A < DISCHARGE_OVER_CURRENT_ALARM_THRESHOLD_01A) {
         outputBits_.alarm.doca = true;
     }


    // Bits Message (0x4250) - Status, Errors, Alarms, Protections
    outputBits_.basic_status.status = determineSystemState(avgCurrent_01mA / 1000);
    // outputBits_.basic_status.forced_charge_request = ?; // Needs external input or logic
    // outputBits_.basic_status.balance_charge_request = ?; // Needs balancing logic
    outputBits_.cycle_period = 0; // Placeholder: Needs definition

    // Fault Extension Info (0x4290) - Needs specific logic or inputs
    // outputFaultExt_.fault_ext1.bmic_error = checkBMICErrors(); // Placeholder

    // Charge/Discharge Status (0x4280) - Already updated by protection checks

    // Log processed values if needed (use LOG_DBG for frequent messages)
    // LOG_DBG("Processing complete. SOC=%u%%, V=%.1fV, I=%.1fA",
    //         outputStatus_.soc, outputStatus_.total_voltage / 10.0, outputStatus_.current / 10.0);

}

// --- Getters ---
 const Message::Status& MasterBMS::getStatus() const { return outputStatus_; }
 const Message::ChargeDischargeParameters& MasterBMS::getChargeDischargeParameters() const { return outputChargeDischargeParams_; }
 const Message::CellVoltageStatus& MasterBMS::getCellVoltageStatus() const { return outputCellVoltageStatus_; }
 const Message::CellTemperatureStatus& MasterBMS::getCellTemperatureStatus() const { return outputCellTemperatureStatus_; }
 const Message::Bits& MasterBMS::getBits() const { return outputBits_; }
 const Message::ModuleVoltageStatus& MasterBMS::getModuleVoltageStatus() const { return outputModuleVoltageStatus_; }
 const Message::ModuleTemperatureStatus& MasterBMS::getModuleTemperatureStatus() const { return outputModuleTemperatureStatus_; }
 const Message::ChargeDischargeStatus& MasterBMS::getChargeDischargeStatus() const { return outputChargeDischargeStatus_; }
 const Message::FaultExtensionInfo& MasterBMS::getFaultExtensionInfo() const { return outputFaultExt_; }

// --- Host Request Handling ---

void MasterBMS::handleHostRequest(Request request) {
    switch (request) {
        case Request::EnsembleInformation:
            { // Add scope for local variables if needed
                LOG_INF("Host requested Ensemble Information. Sending all data via CAN...");
                processData();
                if(!allModulesInitialized_)
                {
                    return;
                }
                // Send Status (0x4210)
                CAN_Send(CAN_ID_STATUS,
                         reinterpret_cast<uint8_t*>(&outputStatus_),
                         sizeof(Message::Status));

                // Send Charge/Discharge Parameters (0x4220)
                CAN_Send(CAN_ID_CHARGE_DISCHARGE_PARAMS,
                         reinterpret_cast<uint8_t*>(&outputChargeDischargeParams_),
                         sizeof(Message::ChargeDischargeParameters));

                // Send Cell Voltage Status (0x4230)
                CAN_Send(CAN_ID_CELL_VOLTAGE_STATUS,
                         reinterpret_cast<uint8_t*>(&outputCellVoltageStatus_),
                         sizeof(Message::CellVoltageStatus));

                // Send Cell Temperature Status (0x4240)
                CAN_Send(CAN_ID_CELL_TEMPERATURE_STATUS,
                         reinterpret_cast<uint8_t*>(&outputCellTemperatureStatus_),
                         sizeof(Message::CellTemperatureStatus));

                // Send Bits (Status, Error, Alarm, Protection) (0x4250)
                CAN_Send(CAN_ID_BITS,
                         reinterpret_cast<uint8_t*>(&outputBits_),
                         sizeof(Message::Bits));

                // Send Module Voltage Status (0x4260)
                CAN_Send(CAN_ID_MODULE_VOLTAGE_STATUS,
                         reinterpret_cast<uint8_t*>(&outputModuleVoltageStatus_),
                         sizeof(Message::ModuleVoltageStatus));

                // Send Module Temperature Status (0x4270)
                CAN_Send(CAN_ID_MODULE_TEMPERATURE_STATUS,
                         reinterpret_cast<uint8_t*>(&outputModuleTemperatureStatus_),
                         sizeof(Message::ModuleTemperatureStatus));

                // Send Charge/Discharge Status (0x4280)
                CAN_Send(CAN_ID_CHARGE_DISCHARGE_STATUS,
                         reinterpret_cast<uint8_t*>(&outputChargeDischargeStatus_),
                         sizeof(Message::ChargeDischargeStatus));

                // Send Fault Extension Info (0x4290)
                CAN_Send(CAN_ID_FAULT_EXTENSION_INFO,
                         reinterpret_cast<uint8_t*>(&outputFaultExt_),
                         sizeof(Message::FaultExtensionInfo));

                LOG_INF("Finished sending Ensemble Information.");
            }
            break;
        case Request::SystemEqipmentInformation:
             // TODO: Implement logic to gather and potentially send system equipment info
             LOG_INF("Host requested System Equipment Information.");
            break;
        default:
            // Handle unknown request
            LOG_WRN("Received unknown host request type: %u", request);
            break;
    }
}

void MasterBMS::worker(k_msgq& queue)
{
    Request rq;
    while(0 == k_msgq_get(&queue, &rq, K_NO_WAIT))
    {
        mGPIO.Toggle(GPIO::Name::LED2);
        handleHostRequest(rq);
    }
}


// --- Private Helper Methods ---


void MasterBMS::resetOutputs() {
     outputStatus_ = {};
     outputChargeDischargeParams_ = {};
     outputCellVoltageStatus_ = {};
     outputCellTemperatureStatus_ = {};
     outputBits_ = {};
     outputModuleVoltageStatus_ = {};
     outputModuleTemperatureStatus_ = {};
     outputChargeDischargeStatus_ = {};
     outputFaultExt_ = {};
     outputBits_.basic_status.status = State::Sleep; // Default state
}


void MasterBMS::checkAllModulesInitialized() {
    if (allModulesInitialized_) return;
    for (size_t i = 0; i < NUM_MODULES; ++i) {
        if (!initializedModules_[i]) {
            return; // Not all initialized yet
        }
    }
    allModulesInitialized_ = true;
    LOG_INF("All %u modules have reported initial data.", NUM_MODULES);
}


bool MasterBMS::checkCommunicationTimeout() {
     int64_t now = k_uptime_get();
     bool timeout_detected = false;
     for (size_t i = 0; i < NUM_MODULES; ++i) {
         if (!initializedModules_[i]) {
             // If never initialized, it's effectively timed out after startup period
             // This case is handled by allModulesInitialized_ check mainly
             continue;
         }
         if ((now - lastUpdateTimeMs_[i]) > MODULE_DATA_TIMEOUT_MS) {
             LOG_ERR("Timeout detected for module %u! Last update %lld ms ago.",
                     i, now - lastUpdateTimeMs_[i]);
             timeout_detected = true;
             // Optional: Reset initialized flag for timed out module?
             // initializedModules_[i] = false;
             // allModulesInitialized_ = false;
         }
     }
     return !timeout_detected; // Return true if communication is OK (no timeouts)
}


uint8_t MasterBMS::calculateSOC(uint16_t min_cell_voltage) {
    // Use min cell voltage for conservative SOC estimation, especially for discharge end
    uint16_t voltage_for_soc = min_cell_voltage;

    // Handle edge cases using LUT boundaries
    if (voltage_for_soc <= socLut[0].voltage_01mV) {
        return socLut[0].soc_percent; // Typically 0%
    }
    if (voltage_for_soc >= socLut[socLutSize - 1].voltage_01mV) {
        return socLut[socLutSize - 1].soc_percent; // Typically 100%
    }

    // Find the segment in the LUT
    for (size_t i = 0; i < socLutSize - 1; ++i) {
        if (voltage_for_soc >= socLut[i].voltage_01mV && voltage_for_soc < socLut[i + 1].voltage_01mV) {
            // Found the segment [i, i+1], interpolate
            return interpolate(voltage_for_soc, socLut[i], socLut[i + 1]);
        }
    }

    // Should not happen if edge cases are handled, but return a safe value if it does
    LOG_WRN("SOC calculation failed to find LUT segment for voltage %u", voltage_for_soc);
    return 50; // Default fallback
}



uint8_t MasterBMS::calculateSOH() {
    // Placeholder: SOH requires tracking capacity fade and/or resistance increase over time.
    // This is complex and requires non-volatile storage and historical data.
    return 99; // Assume good health initially
}


uint16_t MasterBMS::calculateMaxChargeCurrent() {
    // Start with the base maximum charge current allowed by hardware/alarms (absolute value)
    uint16_t base_max_charge_01A = abs(CHARGE_OVER_CURRENT_ALARM_THRESHOLD_01A);
    float min_derating_factor = 1.0f;

    // 1. Temperature Derating (based on existing alarm thresholds)
    if (outputStatus_.temperature < CHARGE_LOW_TEMP_ALARM_THRESHOLD_01C ||
        outputStatus_.temperature > CHARGE_HIGH_TEMP_ALARM_THRESHOLD_01C) {
        // LOG_WRN(...); // Logging moved out or made conditional to reduce spam
        min_derating_factor = fminf(min_derating_factor, TEMP_DERATE_FACTOR);
    }

    // 2. SOC Derating (for Charging)
    uint8_t current_soc = outputStatus_.soc;
    if (current_soc >= 100) {
        min_derating_factor = 0.0f; // No charging at 100% SOC
    } else if (current_soc >= SOC_NEAR_FULL_CHARGE_DERATE_START) {
         min_derating_factor = fminf(min_derating_factor, SOC_NEAR_FULL_CHARGE_FACTOR);
    } else if (current_soc >= SOC_HIGH_CHARGE_DERATE_START) {
        min_derating_factor = fminf(min_derating_factor, SOC_HIGH_CHARGE_FACTOR);
    }
    // else: SOC is below high threshold, factor remains 1.0 for SOC

    // 3. Cell Voltage Imbalance Derating
    uint16_t imbalance_01mV = 0;
    if (outputCellVoltageStatus_.max_cell_voltage >= outputCellVoltageStatus_.min_cell_voltage) {
         imbalance_01mV = outputCellVoltageStatus_.max_cell_voltage - outputCellVoltageStatus_.min_cell_voltage;
    } // else: handle potential wrap-around if max < min somehow? Assume unlikely.
    if (imbalance_01mV > CELL_IMBALANCE_DERATE_THRESHOLD_01MV) {
        min_derating_factor = fminf(min_derating_factor, IMBALANCE_DERATE_FACTOR);
    }

    // 4. SOH Derating
    uint8_t current_soh = outputStatus_.soh;
    if (current_soh < SOH_DERATE_LEVEL3_THRESHOLD) {
        min_derating_factor = fminf(min_derating_factor, SOH_DERATE_LEVEL3_FACTOR);
    } else if (current_soh < SOH_DERATE_LEVEL2_THRESHOLD) {
        min_derating_factor = fminf(min_derating_factor, SOH_DERATE_LEVEL2_FACTOR);
    } else if (current_soh < SOH_DERATE_LEVEL1_THRESHOLD) {
        min_derating_factor = fminf(min_derating_factor, SOH_DERATE_LEVEL1_FACTOR);
    }
     // else: SOH is good, factor remains 1.0 for SOH

    // Apply the most restrictive derating factor
    uint16_t final_max_charge_01A = static_cast<uint16_t>(base_max_charge_01A * min_derating_factor);

    // Log derating if significant (optional)
    if (min_derating_factor < 0.99f) { // Use tolerance for float comparison
        LOG_DBG("Charge current derated: Base=%.1fA, Factor=%.2f, Final=%.1fA",
                base_max_charge_01A / 10.0f, min_derating_factor, final_max_charge_01A / 10.0f);
    }


    return final_max_charge_01A; // Return absolute value in 0.1A
}



uint16_t MasterBMS::calculateMaxDischargeCurrent() {
    // Start with the base maximum discharge current allowed by hardware/alarms (absolute value)
    uint16_t base_max_discharge_01A = abs(DISCHARGE_OVER_CURRENT_ALARM_THRESHOLD_01A);
    float min_derating_factor = 1.0f;

    // 1. Temperature Derating (based on existing alarm thresholds)
    if (outputStatus_.temperature < DISCHARGE_LOW_TEMP_ALARM_THRESHOLD_01C ||
        outputStatus_.temperature > DISCHARGE_HIGH_TEMP_ALARM_THRESHOLD_01C) {
        // LOG_WRN(...); // Logging moved out or made conditional
        min_derating_factor = fminf(min_derating_factor, TEMP_DERATE_FACTOR);
    }

    // 2. SOC Derating (for Discharging)
    uint8_t current_soc = outputStatus_.soc;
    if (current_soc == 0) { // Check explicitly for 0%
        min_derating_factor = 0.0f; // No discharging at 0% SOC
    } else if (current_soc <= SOC_NEAR_EMPTY_DISCHARGE_DERATE_START) {
        min_derating_factor = fminf(min_derating_factor, SOC_NEAR_EMPTY_DISCHARGE_FACTOR);
    } else if (current_soc <= SOC_LOW_DISCHARGE_DERATE_START) {
        min_derating_factor = fminf(min_derating_factor, SOC_LOW_DISCHARGE_FACTOR);
    }
    // else: SOC is above low threshold, factor remains 1.0 for SOC


    // 3. Cell Voltage Imbalance Derating (Same logic as for charge)
    uint16_t imbalance_01mV = 0;
    if (outputCellVoltageStatus_.max_cell_voltage >= outputCellVoltageStatus_.min_cell_voltage) {
         imbalance_01mV = outputCellVoltageStatus_.max_cell_voltage - outputCellVoltageStatus_.min_cell_voltage;
    }
    if (imbalance_01mV > CELL_IMBALANCE_DERATE_THRESHOLD_01MV) {
        min_derating_factor = fminf(min_derating_factor, IMBALANCE_DERATE_FACTOR);
    }

    // 4. SOH Derating (Same logic as for charge)
    uint8_t current_soh = outputStatus_.soh;
    if (current_soh < SOH_DERATE_LEVEL3_THRESHOLD) {
        min_derating_factor = fminf(min_derating_factor, SOH_DERATE_LEVEL3_FACTOR);
    } else if (current_soh < SOH_DERATE_LEVEL2_THRESHOLD) {
        min_derating_factor = fminf(min_derating_factor, SOH_DERATE_LEVEL2_FACTOR);
    } else if (current_soh < SOH_DERATE_LEVEL1_THRESHOLD) {
        min_derating_factor = fminf(min_derating_factor, SOH_DERATE_LEVEL1_FACTOR);
    }
     // else: SOH is good, factor remains 1.0 for SOH

    // Apply the most restrictive derating factor
    uint16_t final_max_discharge_01A = static_cast<uint16_t>(base_max_discharge_01A * min_derating_factor);

     // Log derating if significant (optional)
    if (min_derating_factor < 0.99f) { // Use tolerance for float comparison
        LOG_DBG("Discharge current derated: Base=%.1fA, Factor=%.2f, Final=%.1fA",
                base_max_discharge_01A / 10.0f, min_derating_factor, final_max_discharge_01A / 10.0f);
    }

    return final_max_discharge_01A; // Return absolute value in 0.1A
}


State MasterBMS::determineSystemState(int16_t current_01A) {
    // Idle threshold slightly larger to avoid noise around zero
    constexpr int16_t idle_current_threshold_01A = 2; // 200mA

    // If communication is not OK, force Idle
    if (!communicationOk_) {
        return State::Idle;
    }

    // Check for critical faults first - might force Idle or Sleep
    if (outputBits_.error.other_error /* || any other major fault */ ) {
         // Maybe keep last state if fault is minor? Or force Idle for safety.
         return State::Idle;
    }

    // Check protections - might force Idle
    if (outputChargeDischargeStatus_.charge_forbidden && outputChargeDischargeStatus_.discharge_forbidden) {
         // Cannot do anything, could be Sleep if conditions met, otherwise Idle.
         // TODO: Add specific Sleep logic (e.g. prolonged idle and low SOC)
         return State::Idle;
    }

    if (current_01A > idle_current_threshold_01A) { // Charging
        if (!outputChargeDischargeStatus_.charge_forbidden) {
             return State::Charge;
        } else {
            LOG_WRN("System indicates charging current (%.1f A) but charging is forbidden!", current_01A / 10.0);
            return State::Idle; // Charging but forbidden -> Contactor should open, state becomes Idle.
        }
    } else if (current_01A < -idle_current_threshold_01A) { // Discharging
         if (!outputChargeDischargeStatus_.discharge_forbidden) {
            return State::Discharge;
        } else {
             LOG_WRN("System indicates discharging current (%.1f A) but discharging is forbidden!", current_01A / 10.0);
             return State::Idle; // Discharging but forbidden -> Contactor should open, state becomes Idle.
        }
    } else { // Near zero current
         // TODO: Add logic for Sleep state (e.g., after prolonged idle, low SOC etc.)
         return State::Idle;
    }
}