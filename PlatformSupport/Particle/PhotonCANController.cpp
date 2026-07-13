#include "PhotonCANController.h"

#if PLATFORM_ID == PLATFORM_PHOTON_PRODUCTION

CANChannel can(CAN_D1_D2);

bool PhotonCANController::begin(uint32_t busSpeed) {
    currentBaudRate = busSpeed;
    can.begin(busSpeed);
    return true;
}

bool PhotonCANController::messageAvailable() {
    CANBusMessage message;
    return receive(message);
}

bool PhotonCANController::receive(CANBusMessage &outputMessage) {
    CANMessage inputMessage;
    bool receivedMessage = can.receive(inputMessage);
    if (!receivedMessage) {
        return false;
    }

    if (inputMessage.id == 0) {
        return false;
    }

    outputMessage.addr = inputMessage.id;
    for (uint8_t i = 0; i < 8; ++i) {
        outputMessage.bytes[i] = inputMessage.data[i];
    }
    return true;
}

void PhotonCANController::send(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {
    CANMessage txMessage;
    txMessage.id = addr;
    txMessage.len = 8;
    txMessage.data[0] = data0;
    txMessage.data[1] = data1;
    txMessage.data[2] = data2;
    txMessage.data[3] = data3;
    txMessage.data[4] = data4;
    txMessage.data[5] = data5;
    txMessage.data[6] = data6;
    txMessage.data[7] = data7;
    can.transmit(txMessage);
}

void PhotonCANController::send(CANBusMessage inputMessage) {
    CANMessage txMessage;
    txMessage.id = inputMessage.addr;
    txMessage.len = 8;
    for (uint8_t i = 0; i < 8; ++i) {
        txMessage.data[i] = inputMessage.bytes[i];
    }
    can.transmit(txMessage);
}

void PhotonCANController::setBusSpeed(uint32_t newBusSpeed) {
    currentBaudRate = newBusSpeed;
    can.end();
    can.begin(newBusSpeed);
}

uint32_t PhotonCANController::getBusSpeed() {
    return currentBaudRate;
}

bool PhotonCANController::addFilter(uint32_t address) {
    can.addFilter(address, 0x7FF);
    return true;
}

uint8_t PhotonCANController::getRemainingFilters() {
    return 0;
}

void PhotonCANController::sleep() {
    can.end();
}

void PhotonCANController::wake() {
    can.begin(currentBaudRate);
}

#endif
