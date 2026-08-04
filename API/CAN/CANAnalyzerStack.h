#pragma once

#include "ICANController.h"
#include "../Serial/SerialInterfaceHelpers.h"

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

#define EMULATION_BANK_MSG_COUNT 10

class CANAnalyzerStack {
public:
    /// @brief Construct the analyzer stack with the serial transport, CAN controller, and shared receive buffer.
    CANAnalyzerStack(SerialInterfaceHelpers &serial,
                     ICANController &can_controller,
                     CANBusMessage *rx_buffer,
                     uint16_t rx_buffer_size,
                     uint16_t *rx_buffer_index,
                     uint64_t (*getMillis)());

    /// @brief Process a single console command line and dispatch the appropriate action.
    void processLine(const std::string &line);

    /// @brief Processes CAN frames already receives and prints data to console if necessary.
    void processCANFrames();

    /// @brief Calls receive on CAN controller an stores frames into the shared receive buffer.
    void receiveCANFrames();

private:
    /// @brief Print the available command help text to the console.
    void printHelp() const;

    /// @brief Print the latest frame observed for each CAN ID from the receive buffer.
    void printLatestFrames() const;
    void printDeltaFrames();
    void togglePrintAll();

    /// @brief Parse a string as either hexadecimal or decimal.
    uint32_t parseHexOrDec(const std::string &t) const;
    std::vector<CANBusMessage> getLatestUniqueFrames() const;

    void singlePacketCommand(const std::string &cmd);
    // Tokenize a command string into whitespace-separated tokens.
    void tokenizeCommand(const std::string &cmd, std::vector<std::string> &tokens) const;
    // Command handlers (stubs) for menu commands
    void handleUARTBaud(const std::string &cmd);
    void handleCANBaud(const std::string &cmd);
    void flushBank1();
    void flushBank2();
    void addMsgBank1(const std::string &cmd);
    void addMsgBank2(const std::string &cmd);
    // Helper to add or update a message in an emulation bank
    void addOrUpdateMsgBank(const CANBusMessage &msg, std::vector<CANBusMessage> &bank, uint8_t &bank_index, const char *bankName, uint32_t publish_period_ms = 0);
    void processLoopFrame(uint64_t now);
    void dataLoopMode(const std::string &cmd);
    void addrLoopMode(const std::string &cmd);
    void comboLoopMode(const std::string &cmd);
    void stopLoop();
    void queryMessages() const;
    // Print both emulation banks side-by-side
    void printMessageBanks();

    SerialInterfaceHelpers &serial_;
    ICANController &can_controller_;
    CANBusMessage *rx_buffer_;
    uint16_t rx_buffer_size_;
    uint16_t *rx_buffer_index_;
    bool app_mode_;
    bool print_all_enabled_;
    
    struct LastSeenFrame {
        uint8_t bytes[8];
    };

    std::unordered_map<uint32_t, LastSeenFrame> last_seen_;
    uint16_t last_printed_index_;
    // Platform-specific function to get system ticks in milliseconds.
    uint64_t (*getMillis_)();
    // Helper to call the stored function pointer (returns 0 if not set)
    uint64_t getSystemMillis() const;
    // Emulation banks for continuously-sent messages
    std::vector<CANBusMessage> emulation_bank1_;
    std::vector<CANBusMessage> emulation_bank2_;
    uint8_t bank1_index_;
    uint8_t bank2_index_;

    enum class LoopType : uint8_t {
        None,
        Data,
        Address,
        Combo
    };

    LoopType loop_type_ = LoopType::None;
    CANBusMessage loop_message_;
    uint32_t loop_address_ = 0;
    uint32_t loop_end_address_ = 0;
    uint32_t loop_address_step_ = 1;
    uint16_t loop_data_value_ = 0;
    uint8_t loop_data_mask_ = 0;
};
