#pragma once

#include <stdint.h>
#include "CANBusMessage.h"

/// @brief Platform-agnostic interface for CAN controllers
class ICANController{
    public:

    // ---- INITIALIZATION FUNCTIONS ----

    // Performs startup initialization on the controller (if necessary) and configures bus speed. Return value indicates success
    virtual bool begin(uint32_t busSpeed) = 0;


    // ---- CANBUS SEND/RECEIVE FUNCTIONS ----

    // Checks if a message is available for receiving.
    virtual bool messageAvailable() = 0;

    // Attempts to read a CAN frame from the controller. Return value indicates success
    virtual bool receive(CANBusMessage &outputMessage) = 0;
    
    // Transmits a CAN frame to the bus
    virtual void send(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) = 0;
    
    // Transmits a CAN frame to the bus
    virtual void send(CANBusMessage inputMessage) = 0;


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
};