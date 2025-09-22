#pragma once
#define ARDUINOJSON_ENABLE_PROGMEM 0  //Important: this needs to go before any ArduinoJson includes to disable PROGMEM support
#include <ArduinoJson.h>
#include <string>
#include "../HVBoards/DecentralizedLV-HVBoards.h" // Include for HVController_CAN

class AppStatus {
public:
    // Fields that the App can read   
    // HVController_CAN fields
    bool Killswitch;
    bool BMSFault;
    bool hvBoardDetected;
    bool dischargeContactorOn;
    bool chargeContactorOn;
    bool chargeSafetyOn;
    uint8_t hvPackSOC;
    float hvMotorTemperatureC;
    float hvInverterTemperatureC;
    uint8_t hvThermistorHighTempC;

    // OrionBMS fields
    uint8_t batterySOC;
    float packCurrentAmps;
    float packInstantaneousVoltage;
    float inputSupplyVoltage;
    float avgCellVoltage;
    float highestCellVoltage;
    float lowestCellVoltage;
    float packAmpHours;
    float packResistanceOhms;
    float lowestCellResistanceOhms;
    uint16_t dtcFlags1;
    uint16_t dtcFlags2;
    uint16_t dischargeCurrentLimit;
    uint16_t chargeCurrentLimit;
    uint8_t bmsAverageTempC;
    uint8_t bmsInternalTempC;
    uint8_t thermistorHighTempC;
    uint8_t thermistorLowTempC;
    uint16_t relayState;
    bool j1772PlugState;
    uint8_t j1772ACCurrentLimit;
    uint8_t j1772ACVoltage;

    // Per-cell voltages from OrionBMS
    float cellVoltages[180];

    // RMSController fields
    uint16_t postFaultHigh;
    uint16_t postFaultLow;
    uint16_t runFaultHigh;
    uint16_t runFaultLow;
    float accessoryVoltage;
    float busVoltage;
    float busCurrent;
    float commandedTorque;
    float rmsPhaseACurrent;
    float rmsMotorTemperatureC;
    float rmsInverterTemperatureC;
    uint16_t motorRPM;
    float motorTempC;
    bool faultActive;

    // PowerController_CAN fields
    bool Acc;
    bool Ign;
    bool FullStart;
    uint8_t DriveMode;

    // Fields that the App can set
    bool leftTurnSignal_App;
    bool rightTurnSignal_App;
    bool headlight_App;
    bool highbeam_App;
    bool horn_App;
    bool Acc_AppSet;
    bool Ign_AppSet;
    bool FullStart_AppSet;

    AppStatus();

    void copyFromPowerController(const PowerController_CAN& pc);
    void copyFromHVController(const HVController_CAN& hv);
    void copyFromOrionBMS(const OrionBMS& bms);
    void copyFromRMSController(const RMSController& rms);

    std::string toPowerControllerJSON() const;
    std::string toOrionBMSJSON() const;
    std::string toDashboardJSON() const;
    std::string toRMSJSON() const;
    std::string toCellVoltagesJSON() const; // Serialize cellVoltages[] as JSON array

    bool fromJSON(const std::string& json);
};