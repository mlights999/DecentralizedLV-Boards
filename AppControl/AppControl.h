#pragma once
#define ARDUINOJSON_ENABLE_PROGMEM 0  //Important: this needs to go before any ArduinoJson includes to disable PROGMEM support
#include <ArduinoJson.h>
#include <string>
#include "../HVBoards/DecentralizedLV-HVBoards.h" // Include for HVController_CAN

// Forward declaration
class PowerController_CAN;

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
    bool usingAppControl;

    // Dashboard fields
    uint8_t DriveMode;

    // Merged control fields (hardware OR software)
    bool leftTurnSignal;     // True if either hardware or app has it on
    bool rightTurnSignal;    // True if either hardware or app has it on
    bool headlight;          // True if either hardware or app has it on
    bool highbeam;           // True if either hardware or app has it on
    bool horn;               // True if either hardware or app has it on
    bool hazards;            // True if both turn signals are on
    uint8_t leftTurnPWM;     // PWM value for left turn signal
    uint8_t rightTurnPWM;    // PWM value for right turn signal
    bool Acc;
    bool Ign;
    bool FullStart;

    // Actual current state (merged from manual and app controls)
    bool leftTurnSignal_Current;
    bool rightTurnSignal_Current;
    bool headlight_Current;
    bool highbeam_Current;
    bool horn_Current;

    // Actual current state (merged from manual and app controls)
    bool leftTurnSignal_Current;
    bool rightTurnSignal_Current;
    bool headlight_Current;
    bool highbeam_Current;
    bool horn_Current;

    AppStatus();

    void copyFromPowerController(const PowerController_CAN& pc);
    void copyFromHVController(const HVController_CAN& hv);
    void copyFromOrionBMS(const OrionBMS& bms);
    void copyFromRMSController(const RMSController& rms);
<<<<<<< Updated upstream
    void copyFromDashController(const DashController_CAN& dash);
=======
    void mergeControlStates(const DashController_CAN& dc, const AppController_CAN& ac);
>>>>>>> Stashed changes

    std::string toPowerControllerJSON() const;
    std::string toOrionBMSJSON() const;
    std::string toDashboardJSON() const;
    std::string toRMSJSON() const;
    std::string toCellVoltagesJSON() const;

    bool fromJSON(const std::string& json);
};