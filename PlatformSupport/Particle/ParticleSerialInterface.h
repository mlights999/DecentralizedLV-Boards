#pragma once

#include "Particle.h"
#include "../../API/Serial/SerialInterfaceHelpers.h"

/// @brief Serial interface backed by the Particle DeviceOS Serial object.
class ParticleSerialInterface : public SerialInterfaceHelpers {
public:
    void write(char *string) override;
    char read() override;
    int readBytes(char *buffer, uint32_t maxLength) override;
    bool available() override;
    bool processLines(std::vector<std::string> &outLines) override;

private:
    std::string partialLine_;
};