#include "AppControl.h"
#include "../HVBoards/DecentralizedLV-HVBoards.h" // Include for HVController_CAN

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
    Acc_App(false),
    Ign_App(false),
    FullStart_App(false),
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
    faultActive(false)
{}

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
    if (doc.containsKey("acc")) Acc_App = doc["acc"];
    if (doc.containsKey("ign")) Ign_App = doc["ign"];
    if (doc.containsKey("fs")) FullStart_App = doc["fs"];
    return true;
}

std::string AppStatus::toPowerControllerJSON() const {
    StaticJsonDocument<128> doc;
    doc["type"] = "pc";
    doc["acc"] = Acc_App;
    doc["ign"] = Ign_App;
    doc["fs"] = FullStart_App;
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

std::string AppStatus::toDashboardJSON() const {
    StaticJsonDocument<128> doc;
    doc["type"] = "dash";
    doc["lts"] = leftTurnSignal_App;
    doc["rts"] = rightTurnSignal_App;
    doc["hl"] = headlight_App;
    doc["hb"] = highbeam_App; // Added highbeam field
    doc["hn"] = horn_App;
    doc["acc"] = Acc_App;
    doc["ign"] = Ign_App;
    doc["fs"] = FullStart_App;
    std::string output;
    serializeJson(doc, output);
    return output;
}