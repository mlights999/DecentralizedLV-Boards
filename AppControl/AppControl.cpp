#include "AppControl.h"
#include "../HVBoards/DecentralizedLV-HVBoards.h" // Include for HVController_CAN
#include "../DecentralizedLV-Boards.h" // Include for PowerController_CAN and DashController_CAN

AppStatus::AppStatus() :
    batterySOC(0),
    motorTempC(0.0f),
    Killswitch(false),
    BMSFault(false),
    hvBoardDetected(false),
    dischargeContactorOn(false),
    chargeContactorOn(false),
    chargeSafetyOn(false),
    hvPackSOC(0),
    hvMotorTemperatureC(0.0f),
    hvInverterTemperatureC(0.0f),
    hvThermistorHighTempC(0),
    packCurrentAmps(0.0f),
    packInstantaneousVoltage(0.0f),
    inputSupplyVoltage(0.0f),
    avgCellVoltage(0.0f),
    highestCellVoltage(0.0f),
    lowestCellVoltage(0.0f),
    packAmpHours(0.0f),
    packResistanceOhms(0.0f),
    lowestCellResistanceOhms(0.0f),
    dtcFlags1(0),
    dtcFlags2(0),
    dischargeCurrentLimit(0),
    chargeCurrentLimit(0),
    bmsAverageTempC(0),
    bmsInternalTempC(0),
    thermistorHighTempC(0),
    thermistorLowTempC(0),
    relayState(0),
    j1772PlugState(false),
    j1772ACCurrentLimit(0),
    j1772ACVoltage(0),
    leftTurnSignal_App(false),
    rightTurnSignal_App(false),
    headlight_App(false),
    highbeam_App(false),
    horn_App(false),
    stereo_App(true),            //Default ON
    ipadCharger_App(true),       //Default ON
    telemetry_App(true),         //Default ON
    radio_App(true),             //Default ON
    wiper_App(false),            //Default OFF - wipers should never run on boot
    runningLights_App(true),     //Default ON
    eyesMode_App(false),         //Default OFF - never persisted, so a power cycle always restores normal lighting
    Acc_App(false),
    Ign_App(false),
    FullStart_App(false),
    leftTurnSignal_Current(false),
    rightTurnSignal_Current(false),
    headlight_Current(false),
    highbeam_Current(false),
    horn_Current(false),
    stereo_Current(true),        //Default ON
    ipadCharger_Current(true),   //Default ON
    telemetry_Current(true),     //Default ON
    radio_Current(true),         //Default ON
    wiper_Current(false),        //Default OFF
    runningLights_Current(true), //Default ON
    eyesMode_Current(false),     //Default OFF
    DriveMode(0),
    postFaultHigh(0),
    postFaultLow(0),
    runFaultHigh(0),
    runFaultLow(0),
    accessoryVoltage(0.0f),
    busVoltage(0.0f),
    busCurrent(0.0f),
    commandedTorque(0.0f),
    rmsPhaseACurrent(0.0f),
    rmsMotorTemperatureC(0.0f),
    rmsInverterTemperatureC(0.0f),
    motorRPM(0),
    faultActive(false),
    usingAppControl(false),
    occupantFanSpeed_App(0),
    batteryFanOverride_App(false),      //Default OFF - never persisted, so a power cycle always restores temperature-based control
    batteryFanManualSpeed_App(0),
    ledStripBrightness_App(255),        //Default full brightness
    occupantFanSpeed_Current(0),
    batteryFanPWM(0),
    batteryFanOverride_Current(false),
    ledStripBrightness_Current(255),    //Default full brightness
    odometerMiles(0.0)
{
    // Initialize cell voltages array to 0
    for(int i = 0; i < 180; i++) {
        cellVoltages[i] = 0.0f;
    }
    // Initialize per-board CAN health to 0 (off / not yet heard)
    for(uint8_t i = 0; i < BOARD_STATUS_COUNT; i++) {
        boardStatus[i] = 0;
    }
}

// Removed duplicate earlier definition. Implementation below updates telemetry fields.

void AppStatus::copyFromHVController(const HVController_CAN& hv) {
    Killswitch = hv.Killswitch;
    BMSFault = hv.BMSFault;
    hvBoardDetected = hv.boardDetected;
    dischargeContactorOn = hv.dischargeContactorOn;
    chargeContactorOn = hv.chargeContactorOn;
    chargeSafetyOn = hv.chargeSafetyOn;
    hvPackSOC = hv.packSOC;
    hvMotorTemperatureC = hv.motorTemperatureC;
    hvInverterTemperatureC = hv.inverterTemperatureC;
    hvThermistorHighTempC = hv.thermistorHighTempC;
    // batteryFanPWM (app status) is no longer sourced from the HV Controller - it's computed from the
    // shared battTempToPWM ramp (or the manual override) in PowerController.ino, since the front-left
    // LPDRV board now owns battery-fan control.
}

void AppStatus::copyFromOrionBMS(const OrionBMS& bms) {
    packCurrentAmps = bms.packCurrentAmps;
    packInstantaneousVoltage = bms.packInstantaneousVoltage;
    inputSupplyVoltage = bms.inputSupplyVoltage;
    avgCellVoltage = bms.avgCellVoltage;
    highestCellVoltage = bms.highestCellVoltage;
    lowestCellVoltage = bms.lowestCellVoltage;
    packAmpHours = bms.packAmpHours;
    packResistanceOhms = bms.packResistanceOhms;
    lowestCellResistanceOhms = bms.lowestCellResistanceOhms;
    dtcFlags1 = bms.dtcFlags1;
    dtcFlags2 = bms.dtcFlags2;
    dischargeCurrentLimit = bms.dischargeCurrentLimit;
    chargeCurrentLimit = bms.chargeCurrentLimit;
    bmsAverageTempC = bms.bmsAverageTempC;
    bmsInternalTempC = bms.bmsInternalTempC;
    thermistorHighTempC = bms.thermistorHighTempC;
    thermistorLowTempC = bms.thermistorLowTempC;
    relayState = bms.relayState;
    j1772PlugState = bms.j1772PlugState;
    j1772ACCurrentLimit = bms.j1772ACCurrentLimit;
    j1772ACVoltage = bms.j1772ACVoltage;
    batterySOC = bms.packSOC;
    
    // Copy per-cell voltages array
    for (int i = 0; i < 180; i++) {
        cellVoltages[i] = bms.cellVoltages[i];
    }
}

void AppStatus::copyFromPowerController(const PowerController_CAN& pc) {
    // Copy power controller telemetry for the app
    usingAppControl = pc.usingAppControl;
    Acc = pc.Acc;
    Ign = pc.Ign;
    FullStart = pc.FullStart;
    horn = pc.Horn;
}

void AppStatus::copyFromRMSController(const RMSController& rms) {
    postFaultHigh = rms.postFaultHigh;
    postFaultLow = rms.postFaultLow;
    runFaultHigh = rms.runFaultHigh;
    runFaultLow = rms.runFaultLow;
    accessoryVoltage = rms.accessoryVoltage;
    busVoltage = rms.busVoltage;
    busCurrent = rms.busCurrent;
    commandedTorque = rms.commandedTorque;
    rmsPhaseACurrent = rms.rmsPhaseACurrent;
    rmsMotorTemperatureC = rms.motorTemperatureC;
    rmsInverterTemperatureC = rms.inverterTemperatureC;
    motorRPM = rms.motorRPM;
    faultActive = rms.faultActive;
}

void AppStatus::copyFromDashController(const DashController_CAN& dc) {
    leftTurnSignal_Current = (dc.leftTurnPWM > 0);
    rightTurnSignal_Current = (dc.rightTurnPWM > 0);
    headlight_Current = dc.headlight;
    highbeam_Current = dc.highbeam;
    runningLights_Current = dc.runningLights;
    // Merge dash fault indicators with authoritative HV/RMS sources (FIX: was overwriting with =)
    BMSFault = BMSFault || dc.bmsFaultDetected;
    faultActive = faultActive || dc.rmsFaultDetected;
    DriveMode = dc.driveMode;
}

void AppStatus::mergeControlStates(const DashController_CAN& dc, const AppController_CAN& ac) {
    // Logical OR: if either hardware or software says "on", then it's on
    // Hardware has priority - if it's on, software can't turn it off
    leftTurnSignal_Current = (dc.leftTurnPWM > 0) || ac.leftTurnSignal;
    rightTurnSignal_Current = (dc.rightTurnPWM > 0) || ac.rightTurnSignal;
    headlight_Current = dc.headlight || ac.headlight;
    highbeam_Current = dc.highbeam || ac.highbeam;
    horn_Current = ac.horn || horn;  //FIX: was horn_Current (self-reference, latched on forever). Use hardware horn state from PowerController.
    stereo_Current = ac.stereo;           // App controlled
    ipadCharger_Current = ac.ipadCharger; // App controlled
    telemetry_Current = ac.telemetry;     // App controlled
    radio_Current = ac.radio;             // App controlled
    wiper_Current = ac.wiper;             // App controlled
    runningLights_Current = dc.runningLights; // Dash Controller is source of truth
    eyesMode_Current = ac.eyesMode; // App-only preference, no manual/hardware source - echo straight through
    DriveMode = dc.driveMode;
}

/// @brief Deserialize JSON string to AppStatus object. ONLY PARSE THE FIELDS THAT THE APP CAN SET.
/// @param json JSON string
/// @return true if successful, false otherwise
bool AppStatus::fromJSON(const std::string& json) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) return false;
    if (doc.containsKey("lts")) leftTurnSignal_App = doc["lts"];
    if (doc.containsKey("rts")) rightTurnSignal_App = doc["rts"];
    if (doc.containsKey("hl")) headlight_App = doc["hl"];
    if (doc.containsKey("hb")) highbeam_App = doc["hb"];
    if (doc.containsKey("hn")) horn_App = doc["hn"];
    if (doc.containsKey("st")) stereo_App = doc["st"];
    if (doc.containsKey("ic")) ipadCharger_App = doc["ic"];
    if (doc.containsKey("tel")) telemetry_App = doc["tel"];  // Telemetry radio power
    if (doc.containsKey("rad")) radio_App = doc["rad"];      // Ham/comms radio power
    if (doc.containsKey("wip")) wiper_App = doc["wip"];      // Windshield wiper power
    if (doc.containsKey("rl")) runningLights_App = doc["rl"];
    if (doc.containsKey("em")) eyesMode_App = doc["em"];  // "Eyes" animation override (novelty feature, never persisted)
    if (doc.containsKey("acc")) Acc_App = doc["acc"];
    if (doc.containsKey("ign")) Ign_App = doc["ign"];
    if (doc.containsKey("fs")) FullStart_App = doc["fs"];
    if (doc.containsKey("ofan")) occupantFanSpeed_App = doc["ofan"];  // Occupant-cell fan speed (0-255)
    if (doc.containsKey("bfo")) batteryFanOverride_App = doc["bfo"];  // Battery-box fan manual override (testing/validation only, never persisted)
    if (doc.containsKey("bfm")) batteryFanManualSpeed_App = doc["bfm"];  // Battery-box fan manual speed (0-255) used while override is on
    if (doc.containsKey("lb")) ledStripBrightness_App = doc["lb"];  // Interior dash LED strip brightness (0-255)
    return true;
}

std::string AppStatus::toPowerControllerJSON() const {
    StaticJsonDocument<256> doc;
    doc["type"] = "pc";
    doc["acc"] = Acc;
    doc["ign"] = Ign;
    doc["fs"] = FullStart;
    doc["hn"] = horn_Current;
    doc["uac"] = usingAppControl;
    doc["odo"] = (double)((long)(odometerMiles * 10)) / 10.0;  // miles, 1 decimal
    // Per-board CAN health: [Power, Dash, HV, iBooster, BMS, RMS] (0=off, 1=online, 2=fault)
    JsonArray bs = doc.createNestedArray("bs");
    for(uint8_t i = 0; i < BOARD_STATUS_COUNT; i++) {
        bs.add(boardStatus[i]);
    }
    std::string output;
    serializeJson(doc, output);
    return output;
}

// Split into two smaller packets so Bluetooth 4.0 devices (small ATT MTU) can receive them.
// bms  (type="bms") : all display-critical fields the app already knows how to parse (~155 B)
// bms2 (type="bms2"): supplemental diagnostic fields for future app support (~145 B)

std::string AppStatus::toOrionBMSJSON_1() const {
    StaticJsonDocument<320> doc;
    doc["type"] = "bms";   // Keep as "bms" so the existing app parses it without changes
    doc["soc"]  = batterySOC;
    doc["pca"]  = packCurrentAmps;
    doc["piv"]  = packInstantaneousVoltage;
    doc["acv"]  = avgCellVoltage;
    doc["hcv"]  = highestCellVoltage;
    doc["lcv"]  = lowestCellVoltage;
    doc["pah"]  = packAmpHours;
    doc["ht"]   = thermistorHighTempC;   // Pack high temp
    doc["lt"]   = thermistorLowTempC;    // Pack low temp
    doc["bat"]  = bmsAverageTempC;
    doc["bit"]  = bmsInternalTempC;
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toOrionBMSJSON_2() const {
    StaticJsonDocument<320> doc;
    doc["type"] = "bms2";  //FIX: was "bms" (same as packet 1). Changed to "bms2" so app can distinguish the two packets.
    doc["isv"]  = inputSupplyVoltage;
    doc["pro"]  = packResistanceOhms;
    doc["lcro"] = lowestCellResistanceOhms;
    doc["thtc"] = thermistorHighTempC;
    doc["thlc"] = thermistorLowTempC;
    doc["dtc1"] = dtcFlags1;
    doc["dtc2"] = dtcFlags2;
    doc["dcl"]  = dischargeCurrentLimit;
    doc["ccl"]  = chargeCurrentLimit;
    doc["rs"]   = relayState;
    doc["jps"]  = j1772PlugState;
    doc["jacl"] = j1772ACCurrentLimit;
    doc["jav"]  = j1772ACVoltage;
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toRMSJSON() const {
    StaticJsonDocument<256> doc;
    doc["type"] = "rms";
    doc["pfh"] = postFaultHigh;
    doc["pfl"] = postFaultLow;
    doc["rfh"] = runFaultHigh;
    doc["rfl"] = runFaultLow;
    doc["av"] = accessoryVoltage;
    doc["bv"] = busVoltage;
    doc["bc"] = busCurrent;
    doc["ct"] = commandedTorque;
    doc["pac"] = rmsPhaseACurrent;
    doc["mtc"] = rmsMotorTemperatureC;
    doc["itc"] = rmsInverterTemperatureC;
    doc["rpm"] = motorRPM;
    doc["fa"] = faultActive;
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toDashboardJSON() const {
    StaticJsonDocument<384> doc;
    doc["type"] = "dash";
    // Send the actual current state (merged from manual and app controls)
    doc["lts"] = leftTurnSignal_Current;
    doc["rts"] = rightTurnSignal_Current;
    doc["hl"] = headlight_Current;
    doc["hb"] = highbeam_Current;
    doc["hn"] = horn_Current;
    doc["st"] = stereo_Current;
    doc["ic"] = ipadCharger_Current;
    doc["tel"] = telemetry_Current;  // Telemetry radio state the car accepted (echo of the app's request)
    doc["rad"] = radio_Current;      // Ham/comms radio state the car accepted (echo of the app's request)
    doc["wip"] = wiper_Current;      // Windshield wiper state the car accepted (echo of the app's request)
    doc["rl"] = runningLights_Current;      // Actual running-lights output as reported by the Dash (preference AND car-powered)
    doc["rla"] = runningLights_App;         // DEBUG: running-lights value this gateway last received from the app (before Dash gating). If "rla" tracks the toggle but "rl" doesn't, the break is on the Dash (usingAppControl / car-power gate), not the app link.
    doc["dm"] = DriveMode;  // Include drive mode so app can display gear
    doc["acc"] = Acc;       //FIX: was Acc_App (echoed app command). Now sends actual hardware state.
    doc["ign"] = Ign;       //FIX: was Ign_App
    doc["fs"] = FullStart;  //FIX: was FullStart_App
    doc["ofan"] = occupantFanSpeed_Current;  // Occupant-cell fan speed currently commanded (0-255)
    doc["bfan"] = batteryFanPWM;             // Battery-box fan speed currently driven (0-255). Reflects the manual value while an override is active.
    doc["bfo"] = batteryFanOverride_Current; // Battery-fan manual override state the car accepted (echo of the app's request). Non-persistent: always false after a power cycle.
    doc["em"] = eyesMode_Current;            // "Eyes" animation state the car accepted (echo of the app's request). Non-persistent: always false after a power cycle.
    doc["lb"] = ledStripBrightness_Current;  // Interior dash LED strip brightness currently commanded (0-255)
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toCellVoltagesJSON() const {
    // Send a rotating batch of 18 cells each call (reduced from 36 for Bluetooth 4.0 MTU compatibility)
    constexpr size_t kTotal = 180;
    constexpr size_t kBatch = 18;
    static size_t nextStart = 0; // rotates across calls

    size_t start = nextStart;
    nextStart = (nextStart + kBatch) % kTotal;

    // Capacity: small object + array of 18 floats (~140 bytes serialized)
    StaticJsonDocument<320> doc;
    doc["type"] = "cell";
    doc["frstcll"] = static_cast<uint16_t>(start); // include first cell index
    JsonArray arr = doc.createNestedArray("cv");
    for (size_t i = 0; i < kBatch; ++i) {
        size_t idx = (start + i) % kTotal;
        arr.add(cellVoltages[idx]);
    }
    std::string output;
    serializeJson(doc, output);
    return output;
}

/////////////////////////////////////////////////////////////////////
//  Odometer – non-blocking speed integration
//
//  Converts motorRPM to vehicle speed using the same constant the
//  app already uses (speed_mph = rpm / 68.5) and integrates over
//  the elapsed time to accumulate distance in miles.
//
//  Call once per loop() pass.  Pure arithmetic – no I/O, no delay.
/////////////////////////////////////////////////////////////////////

/// @brief Integrates current motor RPM over a time delta to accumulate
///        distance on the odometer.  Non-blocking (pure math).
/// @param rpm      Current motor RPM (from RMSController via CAN)
/// @param deltaMs  Milliseconds since the last call (e.g. millis() - prev)
void AppStatus::updateOdometer(uint16_t rpm, uint32_t deltaMs) {
    if (rpm == 0 || deltaMs == 0) return;           // stationary – nothing to add

    // Speed conversion: same factor used by the app (PowerControllerBLE.ts)
    //   speed_mph = rpm / 68.5
    const double speedMph = static_cast<double>(rpm) / 68.5;

    // distance = speed (mi/h) * time (h)
    //          = speed * (deltaMs / 3,600,000)
    const double distanceMiles = speedMph * (static_cast<double>(deltaMs) / 3600000.0);

    odometerMiles += distanceMiles;
}