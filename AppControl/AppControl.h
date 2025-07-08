#pragma once
#define ARDUINOJSON_ENABLE_PROGMEM 0  //Important: this needs to go before any ArduinoJson includes to disable PROGMEM support
#include <ArduinoJson.h>
#include <string>

class AppStatus {
public:
    
    // Fields that the App can only read
    uint8_t batterySOC;         // State of Charge (0-100)
    float batteryVoltage;       // In volts
    float motorTempC;           // Motor controller temperature in Celsius

    // Fields that the App can set
    bool leftTurnSignal;
    bool rightTurnSignal;
    bool headlight;
    bool horn;
    uint8_t driveMode;
    bool Acc;
    bool Ign;
    bool FullStart;

    AppStatus();

    // Encode fields to JSON string
    std::string toJSON() const;

    // Decode fields from JSON string
    bool fromJSON(const std::string& json);
};