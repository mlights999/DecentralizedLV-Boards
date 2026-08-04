#pragma once

#include <stdint.h>
#include "CANBusMessage.h"

#define ALLOW_CANFRAME_INJECT
#define LOG_SENT_CAN_MESSAGES

/// @brief Platform-agnostic interface for CAN controllers
class ICANController{
    public:

    enum class CANResult {
        Success = 0,
        Failure = 1,
    };

    virtual ~ICANController() = default;

    // ---- INITIALIZATION FUNCTIONS ----

    // Performs startup initialization on the controller (if necessary) and configures bus speed. Return value indicates success
    virtual bool begin(uint32_t busSpeed) = 0;


    // ---- CANBUS SEND/RECEIVE FUNCTIONS ----

    // Checks if a message is available for receiving.
    virtual bool messageAvailable() = 0;

    // Attempts to read a CAN frame from the controller. Return value indicates success
    virtual bool receive(CANBusMessage &outputMessage) = 0;

    // Attempts to read a CAN frame and append it to a circular receive buffer.
    bool receive(CANBusMessage &outputMessage,
                 CANBusMessage messageQueue[],
                 uint16_t *rxBufferIndex,
                 uint16_t rxBufferSize)
    {
        bool received = receive(outputMessage);
        if (received && messageQueue && rxBufferIndex && rxBufferSize != 0) {
            messageQueue[*rxBufferIndex] = outputMessage;
            *rxBufferIndex = static_cast<uint16_t>((*rxBufferIndex + 1) % rxBufferSize);
        }
        return received;
    }
    
    // Transmits a CAN frame to the bus
    virtual CANResult send(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) = 0;
    
    // Transmits a CAN frame to the bus
    virtual CANResult send(CANBusMessage inputMessage) = 0;


    // ---- CANBUS SPEED FUNCTIONS ----

    // Reinitializes the CAN Controller with a new target bus speed
    virtual void setBusSpeed(uint32_t newBusSpeed) = 0;

    // Gets the current bus speed the controller is configured for
    virtual uint32_t getBusSpeed() = 0;

    // ---- CANBUS RECEIVE FILTER FUNCTIONS ----

    // Attempts to add a new CAN receive filter if there are available slots. Return value indicates success.
    virtual bool addFilter(uint32_t address) = 0;

    // Returns the number of filters remaining on the CAN controller
    virtual uint8_t getRemainingFilters() = 0;

    virtual void injectFakeFrame(const CANBusMessage &message) = 0;
};