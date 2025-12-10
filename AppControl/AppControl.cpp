#include "AppControl.h"
#include "../HVBoards/DecentralizedLV-HVBoards.h" // Include for HVController_CAN
#include "../DecentralizedLV-Boards.h" // Include for PowerController_CAN

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
<<<<<<< Updated upstream
    leftTurnSignal_App(false),
    rightTurnSignal_App(false),
    headlight_App(false),
    highbeam_App(false),
    horn_App(false),
<<<<<<< Updated upstream
    Acc_App(false),
    Ign_App(false),
    FullStart_App(false),
=======
    Acc_AppSet(false),
    Ign_AppSet(false),
    FullStart_AppSet(false),
    leftTurnSignal_Current(false),
    rightTurnSignal_Current(false),
    headlight_Current(false),
    highbeam_Current(false),
    horn_Current(false),
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
    leftTurnSignal(false),
    rightTurnSignal(false),
    headlight(false),
    highbeam(false),
    horn(false),
    hazards(false),
    leftTurnPWM(0),
    rightTurnPWM(0),
    Acc(false),
    Ign(false),
    FullStart(false),
>>>>>>> Stashed changes
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
    DriveMode(0)
{}

void AppStatus::copyFromPowerController(const PowerController_CAN& pc) {
    usingAppControl = pc.usingAppControl;
    Acc = pc.Acc;
    Ign = pc.Ign;
    FullStart = pc.FullStart;
}

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

<<<<<<< Updated upstream
void AppStatus::copyFromDashController(const DashController_CAN& dash) {
    // Copy manual control states from dash controller
    // Note: leftTurnPWM and rightTurnPWM > 0 indicates the blinker is on
    // We'll handle this logic in the main loop where we have more context
=======
void AppStatus::mergeControlStates(const DashController_CAN& dc, const AppController_CAN& ac) {
    // Logical OR: if either hardware or software says "on", then it's on
    // Hardware has priority - if it's on, software can't turn it off
    leftTurnSignal = (dc.leftTurnPWM > 0) || ac.leftTurnSignal;
    rightTurnSignal = (dc.rightTurnPWM > 0) || ac.rightTurnSignal;
    headlight = dc.headlight || ac.headlight;
    highbeam = dc.highbeam || ac.highbeam;
    horn = ac.horn;  // Horn only comes from app controller (PowerController.Horn is separate)
    hazards = leftTurnSignal && rightTurnSignal;
    leftTurnPWM = dc.leftTurnPWM;
    rightTurnPWM = dc.rightTurnPWM;
>>>>>>> Stashed changes
}

/// @brief Deserialize JSON string to AppStatus object. ONLY PARSE THE FIELDS THAT THE APP CAN SET.
/// @param json JSON string
/// @return true if successful, false otherwise
bool AppStatus::fromJSON(const std::string& json) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) return false;
    // Note: These values get merged with hardware states in mergeControlStates
    // For now we just store them temporarily and let the merge happen in the main loop
    if (doc.containsKey("lts")) leftTurnSignal = doc["lts"];
    if (doc.containsKey("rts")) rightTurnSignal = doc["rts"];
    if (doc.containsKey("hl")) headlight = doc["hl"];
    if (doc.containsKey("hb")) highbeam = doc["hb"];
    if (doc.containsKey("hn")) horn = doc["hn"];
    if (doc.containsKey("acc")) Acc = doc["acc"];
    if (doc.containsKey("ign")) Ign = doc["ign"];
    if (doc.containsKey("fs")) FullStart = doc["fs"];
    return true;
}

std::string AppStatus::toPowerControllerJSON() const {
    StaticJsonDocument<128> doc;
    doc["type"] = "pc";
    doc["acc"] = Acc;
    doc["ign"] = Ign;
    doc["fs"] = FullStart;
    doc["hn"] = horn;
    doc["uac"] = usingAppControl;
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toOrionBMSJSON() const {
    StaticJsonDocument<512> doc;
    doc["type"] = "bms";
    doc["pca"] = packCurrentAmps;
    doc["piv"] = packInstantaneousVoltage;
    doc["isv"] = inputSupplyVoltage;
    doc["acv"] = avgCellVoltage;
    doc["hcv"] = highestCellVoltage;
    doc["lcv"] = lowestCellVoltage;
    doc["pah"] = packAmpHours;
    doc["pro"] = packResistanceOhms;
    doc["lcro"] = lowestCellResistanceOhms;
    doc["dtc1"] = dtcFlags1;
    doc["dtc2"] = dtcFlags2;
    doc["dcl"] = dischargeCurrentLimit;
    doc["ccl"] = chargeCurrentLimit;
    doc["bat"] = bmsAverageTempC;
    doc["bit"] = bmsInternalTempC;
    doc["thtc"] = thermistorHighTempC;
    doc["thlc"] = thermistorLowTempC;
    doc["rs"] = relayState;
    doc["jps"] = j1772PlugState;
    doc["jacl"] = j1772ACCurrentLimit;
    doc["jav"] = j1772ACVoltage;
    doc["soc"] = batterySOC;
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

std::string AppStatus::toCellVoltagesJSON() const {
    // Create a JSON array with cell voltages
    // Note: This is a placeholder implementation - adjust based on actual cell voltage data source
    StaticJsonDocument<512> doc;
    doc["type"] = "cells";
    doc["hcv"] = highestCellVoltage;
    doc["lcv"] = lowestCellVoltage;
    doc["acv"] = avgCellVoltage;
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toDashboardJSON() const {
    StaticJsonDocument<256> doc;
    doc["type"] = "dash";
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    doc["lts"] = leftTurnSignal_App;
    doc["rts"] = rightTurnSignal_App;
    doc["hl"] = headlight_App;
    doc["hb"] = highbeam_App; // Added highbeam field
    doc["hn"] = horn_App;
    doc["acc"] = Acc_App;
    doc["ign"] = Ign_App;
    doc["fs"] = FullStart_App;
=======
=======
>>>>>>> Stashed changes
    // Send the actual current state (merged from manual and app controls)
    doc["lts"] = leftTurnSignal_Current;
    doc["rts"] = rightTurnSignal_Current;
    doc["hl"] = headlight_Current;
    doc["hb"] = highbeam_Current;
    doc["hn"] = horn_Current;
=======
    // Merged states (hardware OR software)
    doc["lts"] = leftTurnSignal;
    doc["rts"] = rightTurnSignal;
    doc["hl"] = headlight;
    doc["hb"] = highbeam;
    doc["hn"] = horn;
    doc["haz"] = hazards;
    doc["lts_pwm"] = leftTurnPWM;
    doc["rts_pwm"] = rightTurnPWM;
>>>>>>> Stashed changes
    doc["acc"] = Acc;
    doc["ign"] = Ign;
    doc["fs"] = FullStart;
    doc["dm"] = DriveMode;
<<<<<<< Updated upstream
    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string AppStatus::toCellVoltagesJSON() const {
    // Send a rotating batch of 36 cells each call
    constexpr size_t kTotal = 180;
    constexpr size_t kBatch = 36;
    static size_t nextStart = 0; // rotates across calls

    size_t start = nextStart;
    nextStart = (nextStart + kBatch) % kTotal;

    // Capacity: small object + array of 36 floats
    const size_t cap = 512;
    DynamicJsonDocument doc(cap);
    doc["type"] = "cell";
    doc["frstcll"] = static_cast<uint16_t>(start); // include first cell index
    JsonArray arr = doc.createNestedArray("cv");
    for (size_t i = 0; i < kBatch; ++i) {
        size_t idx = (start + i) % kTotal;
        arr.add(cellVoltages[idx]);
    }
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
    std::string output;
    serializeJson(doc, output);
    return output;
}