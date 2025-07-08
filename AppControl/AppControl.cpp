#include "AppControl.h"

AppStatus::AppStatus() :
    batterySOC(0), batteryVoltage(0.0f), motorTempC(0.0f),
    leftTurnSignal(false), rightTurnSignal(false), headlight(false), horn(false),
    driveMode(0), Acc(false), Ign(false), FullStart(false) {}

/// @brief Serialize AppStatus object to JSON string.
/// @return JSON string
std::string AppStatus::toJSON() const {
    StaticJsonDocument<256> doc;
    doc["batterySOC"] = batterySOC;
    doc["batteryVoltage"] = batteryVoltage;
    doc["motorTempC"] = motorTempC;
    doc["leftTurnSignal"] = leftTurnSignal;
    doc["rightTurnSignal"] = rightTurnSignal;
    doc["headlight"] = headlight;
    doc["horn"] = horn;
    doc["driveMode"] = driveMode;
    doc["Acc"] = Acc;
    doc["Ign"] = Ign;
    doc["FullStart"] = FullStart;
    std::string output;
    serializeJson(doc, output);
    return output;
}

/// @brief Deserialize JSON string to AppStatus object. ONLY PARSE THE FIELDS THAT THE APP CAN SET.
/// @param json JSON string
/// @return true if successful, false otherwise
bool AppStatus::fromJSON(const std::string& json) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) return false;
    leftTurnSignal = doc["leftTurnSignal"] | false;
    rightTurnSignal = doc["rightTurnSignal"] | false;
    headlight = doc["headlight"] | false;
    horn = doc["horn"] | false;
    driveMode = doc["driveMode"] | 0;
    Acc = doc["Acc"] | false;
    Ign = doc["Ign"] | false;
    FullStart = doc["FullStart"] | false;
    return true;
}