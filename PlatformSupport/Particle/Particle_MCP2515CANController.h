#pragma once

#include "Particle.h"
#include <mcp_can.h>
#include "API/CAN/CANMessage.h"
#include "API/CAN/ICANController.h"

/// @brief MCP2515-based CAN controller implementation for Particle boards.
class Particle_MCP2515CANController : public ICANController {
public:
    explicit Particle_MCP2515CANController(uint8_t chipSelectPin = SS);

    bool begin(uint32_t busSpeed) override;
    bool messageAvailable() override;
    bool receive(CANMessage &outputMessage) override;
    void send(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) override;
    void send(CANMessage inputMessage) override;
    void setBusSpeed(uint32_t newBusSpeed) override;
    uint32_t getBusSpeed() override;
    bool addFilter(uint32_t address) override;
    uint8_t getRemainingFilters() override;

    void sleep();
    void wake();

private:
    static uint32_t convertBaudRateToMCP(uint32_t baudRate);

    MCP_CAN *canController;
    uint8_t filterIndex;
    uint8_t csPin;
    uint32_t currentBaudRate;
};
