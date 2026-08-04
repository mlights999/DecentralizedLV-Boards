#include "Particle_MCP2515CANController.h"
#include "../../DecentralizedLV-Boards.h"

Particle_MCP2515CANController::Particle_MCP2515CANController(uint8_t chipSelectPin)
    : canController(nullptr), filterIndex(0), csPin(chipSelectPin), currentBaudRate(0) {
}

Particle_MCP2515CANController::~Particle_MCP2515CANController() {
    delete canController;
}

bool Particle_MCP2515CANController::begin(uint32_t busSpeed) {
    delete canController;
    canController = new MCP_CAN(csPin);
    currentBaudRate = convertBaudRateToParticle(busSpeed);
    const uint8_t initResult = canController->begin(MCP_STDEXT, convertBaudRateToMCP(currentBaudRate), MCP_8MHZ);
    if (initResult != CAN_OK) {
        delete canController;
        canController = nullptr;
        return false;
    }
    canController->setMode(MCP_NORMAL);
    SPI.setClockSpeed(8000000);
    filterIndex = 0;
    hasInjectedMessage = false;
    return true;
}

bool Particle_MCP2515CANController::messageAvailable() {
    if (hasInjectedMessage) {
        return true;
    }
    return canController != nullptr && canController->checkReceive() == CAN_MSGAVAIL;
}

bool Particle_MCP2515CANController::receive(CANBusMessage &outputMessage) {
    if (hasInjectedMessage) {
        outputMessage = injectedMessage;
        hasInjectedMessage = false;
        return true;
    }

    if (!messageAvailable()) {
        return false;
    }

    uint32_t rxId = 0;
    unsigned char len = 0;
    unsigned char rxBuf[8] = {0};
    if (canController->readMsgBuf(&rxId, &len, rxBuf) != CAN_OK) {
        return false;
    }

    if (rxId == 0) {
        return false;
    }

    outputMessage.addr = rxId;
    for (uint8_t i = 0; i < 8; ++i) {
        outputMessage.bytes[i] = 0;
    }
    if (len > 8) {
        len = 8;
    }
    for (uint8_t i = 0; i < len; ++i) {
        outputMessage.bytes[i] = rxBuf[i];
    }
    return true;
}

ICANController::CANResult Particle_MCP2515CANController::send(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {
    if (!canController) {
        return ICANController::CANResult::Failure;
    }

    byte data[8] = {data0, data1, data2, data3, data4, data5, data6, data7};
    return canController->sendMsgBuf(addr, 0, 8, data) == CAN_OK
        ? ICANController::CANResult::Success
        : ICANController::CANResult::Failure;
}

ICANController::CANResult Particle_MCP2515CANController::send(CANBusMessage inputMessage) {
    if (!canController) {
        return ICANController::CANResult::Failure;
    }

    byte data[8] = {inputMessage.bytes[0], inputMessage.bytes[1], inputMessage.bytes[2], inputMessage.bytes[3], inputMessage.bytes[4], inputMessage.bytes[5], inputMessage.bytes[6], inputMessage.bytes[7]};
    return canController->sendMsgBuf(inputMessage.addr, 0, 8, data) == CAN_OK
        ? ICANController::CANResult::Success
        : ICANController::CANResult::Failure;
}

void Particle_MCP2515CANController::setBusSpeed(uint32_t newBusSpeed) {
    if (!canController) {
        return;
    }
    const uint32_t particleBaudRate = convertBaudRateToParticle(newBusSpeed);
    if (canController->begin(MCP_STDEXT, convertBaudRateToMCP(particleBaudRate), MCP_8MHZ) == CAN_OK) {
        currentBaudRate = particleBaudRate;
        canController->setMode(MCP_NORMAL);
    }
}

uint32_t Particle_MCP2515CANController::getBusSpeed() {
    return currentBaudRate;
}

bool Particle_MCP2515CANController::addFilter(uint32_t address) {
    if (!canController) {
        return false;
    }

    if (filterIndex == 0) {
        canController->init_Mask(0, 0x01FFC000);
        canController->init_Mask(1, 0x01FFC000);
    }

    if (filterIndex < 6) {
        canController->init_Filt(filterIndex, 0, address << 16);
        filterIndex++;
        return true;
    }

    return false;
}

uint8_t Particle_MCP2515CANController::getRemainingFilters() {
    return filterIndex < 6 ? static_cast<uint8_t>(6 - filterIndex) : 0;
}

void Particle_MCP2515CANController::sleep() {
    if (!canController) {
        return;
    }
    canController->setMode(MCP_SLEEP);
}

void Particle_MCP2515CANController::wake() {
    if (!canController) {
        return;
    }
    canController->setMode(MCP_NORMAL);
}

void Particle_MCP2515CANController::injectFakeFrame(const CANBusMessage &message) {
    injectedMessage = message;
    hasInjectedMessage = true;
}

uint32_t Particle_MCP2515CANController::convertBaudRateToMCP(uint32_t baudRate) {
    if(baudRate > CAN_1000KBPS){   //Check if this is a MCP_CAN_RK baud rate     
        switch (baudRate){
        case 1000000:
            return CAN_1000KBPS;
        case 500000:
            return CAN_500KBPS;
        case 250000:
            return CAN_250KBPS;
        case 200000:
            return CAN_200KBPS;
        case 125000:
            return CAN_125KBPS;
        case 100000:
            return CAN_100KBPS;
        case 50000:
            return CAN_50KBPS;
        default:
            return CAN_500KBPS;  //Use 500kbps CAN if using unrecognized macro
        }
    }
    return baudRate;            //If baudRate <= CAN_1000KBPS, then assumes we're already in MCP format
}
