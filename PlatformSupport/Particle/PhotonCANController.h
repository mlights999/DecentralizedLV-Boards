#pragma once

#include "Particle.h"
#include "../../API/CAN/CANBusMessage.h"
#include "../../API/CAN/ICANController.h"

#if PLATFORM_ID == PLATFORM_PHOTON_PRODUCTION

/// @brief Built-in CAN controller implementation for Particle Photon boards.
class PhotonCANController : public ICANController {
public:
    bool begin(uint32_t busSpeed) override;
    bool messageAvailable() override;
    bool receive(CANBusMessage &outputMessage) override;
    CANResult send(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) override;
    CANResult send(CANBusMessage inputMessage) override;
    void setBusSpeed(uint32_t newBusSpeed) override;
    uint32_t getBusSpeed() override;
    bool addFilter(uint32_t address) override;
    uint8_t getRemainingFilters() override;
    void injectFakeFrame(const CANBusMessage &message) override;

    void sleep();
    void wake();

private:
    uint32_t currentBaudRate = 0;
    bool pendingMessage = false;
    CANBusMessage pendingFrame;
    bool hasInjectedMessage = false;
    CANBusMessage injectedMessage;
};

#endif