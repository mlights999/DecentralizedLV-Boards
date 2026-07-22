#pragma once
#define ARDUINOJSON_ENABLE_PROGMEM 0  //Important: this needs to go before any ArduinoJson includes to disable PROGMEM support
#include <ArduinoJson.h>
#include <string>
#include "../HVBoards/DecentralizedLV-HVBoards.h" // Include for HVController_CAN

// Forward declarations
class PowerController_CAN;
class DashController_CAN;

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
    
    // Cell voltages array for toCellVoltagesJSON
    float cellVoltages[180];  // Array to store individual cell voltages

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

    // Per-board CAN liveness the app renders in the low-voltage board grid + "Board Fault" alert.
    // Order: [Power, Dash, HV, iBooster, BMS, RMS]. Values: 0=off/not-yet-heard, 1=online, 2=fault.
    // Set by the Power Controller's updateBoardHealthStatus(); serialized as "bs" in the "pc" packet.
    static const uint8_t BOARD_STATUS_COUNT = 6;
    uint8_t boardStatus[BOARD_STATUS_COUNT];

    // PowerController_CAN fields
    bool usingAppControl;
    // Telemetry of PowerController state
    bool Acc;
    bool Ign;
    bool FullStart;
    bool horn;

    // Fields that the App can set
    bool leftTurnSignal_App;
    bool rightTurnSignal_App;
    bool headlight_App;
    bool highbeam_App;
    bool horn_App;
    bool stereo_App;         // Stereo state set by app (default ON)
    bool ipadCharger_App;    // iPad charger state set by app (default ON)
    bool telemetry_App;      // Telemetry radio power set by app (default ON). Parsed from JSON key "tel".
    bool radio_App;          // Ham/comms radio power set by app (default ON). Parsed from JSON key "rad".
    bool wiper_App;          // Windshield wiper power set by app (default OFF). Parsed from JSON key "wip".
    bool runningLights_App;  // Running lights preference set by app (default ON). Dash Controller persists this across reboots.
    bool eyesMode_App;      // "Eyes" animation override requested by the app. Parsed from JSON key "em". NOVELTY FEATURE - never persisted, always defaults false on boot so a power cycle always turns it off. Takes priority over all other front-grid lighting on BDFL while true.
    bool Acc_App;
    bool Ign_App;
    bool FullStart_App;
    uint8_t occupantFanSpeed_App;   // Occupant-cell (front cabin) fan speed requested by the app, 0 (off) .. 255 (max). Parsed from JSON key "ofan".
    bool batteryFanOverride_App;    // Manual battery-box fan override requested by the app. Parsed from JSON key "bfo". TESTING/VALIDATION ONLY - never persisted, defaults false on boot so the pack always falls back to temperature-based control after a power cycle.
    uint8_t batteryFanManualSpeed_App;  // Battery-box fan speed to command while batteryFanOverride_App is true, 0 (off) .. 255 (max). Parsed from JSON key "bfm".
    uint8_t ledStripBrightness_App;  // Interior dash LED strip brightness requested by the app, 0 (off) .. 255 (max). Defaults to 255 (full) on boot. Parsed from JSON key "lb".

    // Actual current state (merged from manual and app controls)
    bool leftTurnSignal_Current;
    bool rightTurnSignal_Current;
    bool headlight_Current;
    bool highbeam_Current;
    bool horn_Current;
    bool stereo_Current;        // Current stereo state
    bool ipadCharger_Current;   // Current iPad charger state
    bool telemetry_Current;     // Current telemetry radio state, echoed back to the app (JSON key "tel")
    bool radio_Current;         // Current ham/comms radio state, echoed back to the app (JSON key "rad")
    bool wiper_Current;         // Current windshield wiper state, echoed back to the app (JSON key "wip")
    bool runningLights_Current; // Current running lights state, as reported by the Dash Controller (source of truth)
    bool eyesMode_Current;      // "Eyes" animation state the car actually accepted, echoed back to the app (JSON key "em") so the toggle reflects what's really happening. Non-persistent: always false after a power cycle.
    uint8_t DriveMode;          // Current drive mode
    uint8_t occupantFanSpeed_Current;   // Occupant-cell fan speed actually being commanded (0-255). Echoed back to the app for display.
    uint8_t batteryFanPWM;              // Battery-box fan speed the HV Controller is currently driving (0-255). Read-only status for the app. Reflects the manual value while an override is active, otherwise the temperature-based value.
    bool batteryFanOverride_Current;    // Battery-fan override state the gateway is actually commanding over CAN. Echoed back to the app (JSON key "bfo") so the toggle reflects what the car accepted.
    uint8_t ledStripBrightness_Current; // Interior dash LED strip brightness the Dash Controller actually accepted (0-255). Echoed back to the app (JSON key "lb").

    // ── Odometer ──────────────────────────────────────────────────────────
    //  Accumulated distance in miles, calculated from motorRPM.
    //  Persists across power cycles via EEPROM (see PowerController.ino).
    //  Resolution: ~0.001 mi. Sent to the app in the "pc" JSON packet.
    double odometerMiles;

    AppStatus();

    void copyFromPowerController(const PowerController_CAN& pc);
    void copyFromHVController(const HVController_CAN& hv);
    void copyFromOrionBMS(const OrionBMS& bms);
    void copyFromRMSController(const RMSController& rms);
    void copyFromDashController(const DashController_CAN& dc);
    void mergeControlStates(const DashController_CAN& dc, const AppController_CAN& ac);

    std::string toPowerControllerJSON() const;
    std::string toOrionBMSJSON_1() const;  // Core electrical: pca, piv, isv, acv, hcv, lcv, pah, pro, lcro, soc
    std::string toOrionBMSJSON_2() const;  // Flags, limits, temps, relay: dtc1/2, dcl, ccl, temps, J1772
    std::string toDashboardJSON() const;
    std::string toRMSJSON() const;
    std::string toCellVoltagesJSON() const;

    /// Update the odometer by integrating motorRPM over a time delta.
    /// Call once per loop iteration.  Non-blocking; pure arithmetic.
    /// @param rpm      Current motor RPM (from RMSController)
    /// @param deltaMs  Milliseconds elapsed since the last call
    void updateOdometer(uint16_t rpm, uint32_t deltaMs);

    bool fromJSON(const std::string& json);
};