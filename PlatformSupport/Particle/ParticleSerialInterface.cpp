#include "ParticleSerialInterface.h"

void ParticleSerialInterface::write(char *string) {
    if (string) {
        Serial.write(string);
    }
}

char ParticleSerialInterface::read() {
    if (!Serial.available()) {
        return 0;
    }
    return static_cast<char>(Serial.read());
}

int ParticleSerialInterface::readBytes(char *buffer, uint32_t maxLength) {
    if (!buffer || maxLength == 0) {
        return 0;
    }

    uint32_t bytesRead = 0;
    while (bytesRead < maxLength && Serial.available()) {
        buffer[bytesRead++] = static_cast<char>(Serial.read());
    }
    return static_cast<int>(bytesRead);
}

bool ParticleSerialInterface::available() {
    return Serial.available() > 0;
}

bool ParticleSerialInterface::processLines(std::vector<std::string> &outLines) {
    outLines.clear();

    while (Serial.available()) {
        const char character = static_cast<char>(Serial.read());
        if (character == '\n' || character == '\r') {
            if (!partialLine_.empty()) {
                outLines.push_back(partialLine_);
                partialLine_.clear();
            }
        } else {
            partialLine_.push_back(character);
        }
    }

    return !outLines.empty();
}