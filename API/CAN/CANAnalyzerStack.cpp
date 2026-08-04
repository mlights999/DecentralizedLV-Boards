#include "CANAnalyzerStack.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

std::string formatMessage(const CANBusMessage &message) {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "0x%03X: %02X %02X %02X %02X %02X %02X %02X %02X",
                  static_cast<unsigned int>(message.addr), message.bytes[0], message.bytes[1],
                  message.bytes[2], message.bytes[3], message.bytes[4], message.bytes[5],
                  message.bytes[6], message.bytes[7]);
    return std::string(buffer);
}

}

CANAnalyzerStack::CANAnalyzerStack(SerialInterfaceHelpers &serial,
                                   ICANController &can_controller,
                                   CANBusMessage *rx_buffer,
                                   uint16_t rx_buffer_size,
                                   uint16_t *rx_buffer_index,
                                   uint64_t (*getMillis)())
    : serial_(serial),
      can_controller_(can_controller),
      rx_buffer_(rx_buffer),
      rx_buffer_size_(rx_buffer_size),
      rx_buffer_index_(rx_buffer_index),
            app_mode_(false),
            print_all_enabled_(false),
            getMillis_(getMillis),
            emulation_bank1_(EMULATION_BANK_MSG_COUNT),
            emulation_bank2_(EMULATION_BANK_MSG_COUNT),
            bank1_index_(0),
            bank2_index_(0) {
    serial_.writeLine("CANalyzer Neo console ready");
    serial_.writeLine("Type 'h' for help");
}

uint64_t CANAnalyzerStack::getSystemMillis() const {
        if (getMillis_) return getMillis_();
        return 0ULL;
}

void CANAnalyzerStack::processLine(const std::string &line) {
    if (line.empty()) {
        return;
    }

    std::string trimmed = line;
    while (!trimmed.empty() && std::isspace(static_cast<unsigned char>(trimmed.front()))) {
        trimmed.erase(trimmed.begin());
    }
    while (!trimmed.empty() && std::isspace(static_cast<unsigned char>(trimmed.back()))) {
        trimmed.pop_back();
    }

    if (trimmed.empty()) {
        return;
    }

    const char cmd = trimmed[0];
    switch (cmd) {
        case 'h':
            printHelp();
            break;
        case 'i':
            app_mode_ = true;
            serial_.writeLine("App mode enabled");
            break;
        case 'j':
            app_mode_ = false;
            serial_.writeLine("Standard mode enabled");
            break;
        case 'p':
            printLatestFrames();
            break;
        case 'd':
            printDeltaFrames();
            break;
        case 'v':
            serial_.writeLine("CANalyzer Neo v0.1");
            break;
        case 's':
            singlePacketCommand(trimmed);
            break;
        case 'a':
            togglePrintAll();
            break;
        case 'c':
            handleCANBaud(trimmed);
            break;
        case 'f':
            flushBank1();
            break;
        case 'g':
            flushBank2();
            break;
        case 'm':
            addMsgBank1(trimmed);
            break;
        case 'n':
            addMsgBank2(trimmed);
            break;
        case 'l':
            dataLoopMode(trimmed);
            break;
        case 'o':
            addrLoopMode(trimmed);
            break;
        case 'k':
            comboLoopMode(trimmed);
            break;
        case 'x':
            stopLoop();
            break;
        default:
            serial_.writeLine("Unknown command");
            break;
    }
}

void CANAnalyzerStack::processCANFrames() {
    if(print_all_enabled_) {
        if (rx_buffer_ && rx_buffer_index_ && rx_buffer_size_ != 0) {
            // Ensure last_printed_index_ is within bounds
            if (*rx_buffer_index_ < last_printed_index_) {
                last_printed_index_ = *rx_buffer_index_;
            }

            for (uint16_t i = last_printed_index_; i < *rx_buffer_index_; ++i) {
                const CANBusMessage &msg = rx_buffer_[i];
                serial_.writeLine(formatMessage(msg));
            }

            // Update last_printed_index_
            last_printed_index_ = *rx_buffer_index_;
        }
    }

    // Periodic message emission from emulation banks
    uint64_t now = getSystemMillis();
    processLoopFrame(now);

    // Iterate both emulation banks via pointer to avoid duplicate code
    std::vector<CANBusMessage>* banks[2] = { &emulation_bank1_, &emulation_bank2_ };
    for (int bi = 0; bi < 2; ++bi) {
        std::vector<CANBusMessage> &bank = *banks[bi];
        for (size_t i = 0; i < bank.size(); ++i) {
            CANBusMessage &m = bank[i];
            if (m.addr == 0) continue;
            uint32_t period = m.publish_period_ms;
            if (period == 0) continue;
            uint64_t last = m.last_published_time_ms;
            if ((now - last) >= static_cast<uint64_t>(period)) {
                const auto result = can_controller_.send(m);
                m.last_published_time_ms = now;
                if (result != ICANController::CANResult::Success) {
                    serial_.writeLinef("ERR: CAN transmit failed for 0x%03X", static_cast<unsigned int>(m.addr));
                }
            }
        }
    }
}

void CANAnalyzerStack::processLoopFrame(uint64_t now) {
    if (loop_type_ == LoopType::None || loop_message_.publish_period_ms == 0) {
        return;
    }

    if ((now - loop_message_.last_published_time_ms) < loop_message_.publish_period_ms) {
        return;
    }

    if (loop_type_ == LoopType::Data || loop_type_ == LoopType::Combo) {
        for (uint8_t byte = 0; byte < 8; ++byte) {
            if ((loop_data_mask_ & (1U << byte)) != 0) {
                loop_message_.bytes[byte] = static_cast<uint8_t>(loop_data_value_);
            }
        }
    }

    const uint32_t remaining_addresses = loop_end_address_ - loop_address_ + 1;
    const uint32_t frames_in_batch =
        (loop_type_ == LoopType::Data) ? 1 :
        ((loop_address_step_ < remaining_addresses) ? loop_address_step_ : remaining_addresses);

    for (uint32_t offset = 0; offset < frames_in_batch; ++offset) {
        loop_message_.addr = loop_address_ + offset;
        const auto result = can_controller_.send(loop_message_);
        if (result != ICANController::CANResult::Success) {
            serial_.writeLinef("ERR: CAN transmit failed for loop frame 0x%03X",
                               static_cast<unsigned int>(loop_message_.addr));
        }
    }

    loop_message_.last_published_time_ms = now;

    if (loop_type_ == LoopType::Data) {
        ++loop_data_value_;
        if (loop_data_value_ > 0xFF) {
            loop_type_ = LoopType::None;
        }
    } else if (loop_type_ == LoopType::Address) {
        if (frames_in_batch == remaining_addresses) {
            loop_type_ = LoopType::None;
        } else {
            loop_address_ += frames_in_batch;
        }
    } else if (loop_type_ == LoopType::Combo) {
        ++loop_data_value_;
        if (loop_data_value_ > 0xFF) {
            loop_data_value_ = 0;
            if (frames_in_batch == remaining_addresses) {
                loop_type_ = LoopType::None;
            } else {
                loop_address_ += frames_in_batch;
            }
        }
    }
}

void CANAnalyzerStack::receiveCANFrames() {
    if (!rx_buffer_ || !rx_buffer_index_ || rx_buffer_size_ == 0) {
        return;
    }

    CANBusMessage msg;
    while (can_controller_.receive(msg, rx_buffer_, rx_buffer_index_, rx_buffer_size_)) {
    }
}

void CANAnalyzerStack::printHelp() const {
    serial_.writeLinef("=================================================================================================================");
    serial_.writeLinef("= Avaliable Commands   | Arguments (d = dec, x = hex) | Info                                                    =");
    serial_.writeLinef("= 'h' - Help           | h - - - - - - - - -          | Prints this window                                      =");
    serial_.writeLinef("= 'c' - CAN Baud       | c d - - - - - - - -          | Sets CAN baud to value specified by d0                  =");
    serial_.writeLinef("= 'f' - Flush Messages | f - - - - - - - - -          | Clears Bank 1 messages being sent                       =");
    serial_.writeLinef("= 'g' - Flush Messages | g - - - - - - - - -          | Clears Bank 2 messages being sent                       =");
    serial_.writeLinef("= 'a' - Print all      | a - - - - - - - - -          | Enables printing all received CAN messages (no filter)  =");
    serial_.writeLinef("= 'p' - Print Unique   | p - - - - - - - - -          | Prints list of messages and newest data                 =");
    serial_.writeLinef("= 'd' - Print Changed  | d - - - - - - - - -          | Same as 'p', but only messages that have changed        =");
    serial_.writeLinef("= 's' - Send Message   | s x x x x x x x x x          | Sends one message on the CAN bus                        =");
    serial_.writeLinef("= 'm' - Add Msg Bank 1 | m x x x x x x x x x          | Continuously sent message for Bank 1                    =");
    serial_.writeLinef("= 'n' - Add Msg Bank 2 | n x x x x x x x x x          | Continuously sent message for Bank 2                    =");
    serial_.writeLinef("= 'l' - Data Loop Mode | l x x x - - - - - -          | Loops data 0-255 on addr x0, mask x1, delay x2          =");
    serial_.writeLinef("= 'o' - Addr Loop Mode | o x x x x x - - - -          | Loops addr x0 to x1, data x2, step x3, delay x4         =");
    serial_.writeLinef("= 'k' - Combo Loop Md. | k x x x x - - - - -          | Loops addr x0 to x1 and data 0-xFF, delay x2, step x3   =");
    serial_.writeLinef("= 'x' - Stop Loop      | x - - - - - - - - -          | Stops the active loop                                   =");
    serial_.writeLinef("= 'v' - Version        | v - - - - - - - - -          | Prints version of the firmware on this device           =");
    serial_.writeLinef("= 'i' - En. App Mode   | i - - - - - - - - -          | Switches format of messages to be easy to read by app   =");
    serial_.writeLinef("= 'j' - Dis. App Mode  | j - - - - - - - - -          | Switches format of messages to be easy to read by human =");
    serial_.writeLinef("=================================================================================================================");
}

void CANAnalyzerStack::printLatestFrames() const {
    auto latest_frames = getLatestUniqueFrames();

    if (latest_frames.empty()) {
        serial_.writeLine("No CAN data received yet");
        return;
    }

    for (const CANBusMessage &msg : latest_frames) {
        serial_.writeLine(formatMessage(msg));
    }
}

void CANAnalyzerStack::printDeltaFrames() {
    auto latest_frames = getLatestUniqueFrames();

    if (latest_frames.empty()) {
        serial_.writeLine("No CAN data received yet");
        return;
    }

    for (const CANBusMessage &msg : latest_frames) {
        bool changed = false;

        auto it = last_seen_.find(msg.addr);
        if (it == last_seen_.end()) {
            // First time seeing this ID → treat as changed
            changed = true;
        } else {
            // Compare bytes
            for (int i = 0; i < 8; ++i) {
                if (msg.bytes[i] != it->second.bytes[i]) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed) {
            serial_.writeLine(formatMessage(msg));
        }

        // Update last-seen record
        LastSeenFrame rec;
        for (int i = 0; i < 8; ++i) rec.bytes[i] = msg.bytes[i];
        last_seen_[msg.addr] = rec;
    }
}

void CANAnalyzerStack::togglePrintAll() {
    if(rx_buffer_index_ != nullptr) last_printed_index_ = *rx_buffer_index_;
    print_all_enabled_ = !print_all_enabled_;
    if(print_all_enabled_) serial_.writeLine("Enabling print all");
    else serial_.writeLine("Disabling print all");
}

uint32_t CANAnalyzerStack::parseHexOrDec(const std::string &t) const {
    if (t.rfind("0x", 0) == 0 || t.find_first_of("ABCDEFabcdef") != std::string::npos) {
        return static_cast<uint32_t>(std::strtoul(t.c_str(), nullptr, 16));
    }
    return static_cast<uint32_t>(std::strtoul(t.c_str(), nullptr, 10));
}

std::vector<CANBusMessage> CANAnalyzerStack::getLatestUniqueFrames() const {
    std::vector<CANBusMessage> frames;
    std::vector<uint32_t> ids;

    if (!rx_buffer_ || !rx_buffer_index_ || rx_buffer_size_ == 0) {
        return frames;
    }

    const uint16_t count = (*rx_buffer_index_ == 0) ? rx_buffer_size_ : *rx_buffer_index_;

    for (uint16_t i = 0; i < count; ++i) {
        const uint16_t index = (rx_buffer_size_ + (*rx_buffer_index_ - 1 - i)) % rx_buffer_size_;
        const CANBusMessage &msg = rx_buffer_[index];

        // Skip empty frames
        bool empty = (msg.addr == 0);
        for (int k = 0; k < 8 && !empty; ++k) {
            if (msg.bytes[k] != 0) break;
            if (k == 7) empty = true;
        }
        if (empty) continue;

        // Skip duplicates
        if (std::find(ids.begin(), ids.end(), msg.addr) != ids.end()) {
            continue;
        }

        ids.push_back(msg.addr);
        frames.push_back(msg);
    }

    std::sort(frames.begin(), frames.end(),
              [](const CANBusMessage &a, const CANBusMessage &b) {
                  return a.addr < b.addr;
              });

    return frames;
}

void CANAnalyzerStack::singlePacketCommand(const std::string &cmd)
{
    // Tokenize the string: "s 123 01 02 03 04 05 06 07 08"
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);

    // Expect: s addr b0 b1 b2 b3 b4 b5 b6 b7  → 10 tokens
    if (tokens.size() != 10) {
        serial_.writeLine("ERR: expected 9 arguments after 's'");
        return;
    }

    uint32_t addr = parseHexOrDec(tokens[1]);
    uint8_t data[8];

    for (int i = 0; i < 8; ++i) {
        data[i] = static_cast<uint8_t>(parseHexOrDec(tokens[2 + i]) & 0xFF);
    }

    CANBusMessage msg;
    msg.addr = addr;
    for (int i = 0; i < 8; ++i) {
        msg.bytes[i] = data[i];
    }

    // Send the CAN frame
    can_controller_.send(msg);

    if (app_mode_) {
        serial_.writeLine(formatMessage(msg));
    } else {
        serial_.writeLine(formatMessage(msg));
    }
}

// --------------------------------------------------------------------------
// Stub implementations for commands referenced in printHelp()
// These are intentionally minimal; implement behavior as needed.
// --------------------------------------------------------------------------
void CANAnalyzerStack::handleUARTBaud(const std::string &cmd) {
    (void)cmd;
    serial_.writeLine("Not implemented: set UART baud");
}

void CANAnalyzerStack::handleCANBaud(const std::string &cmd) {
    // Tokenize input (same style as other command parsers)
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);

    if (tokens.size() < 2) {
        // No argument: print current CAN baud
        uint32_t cur = can_controller_.getBusSpeed();
        serial_.writeLinef("CAN baud rate: %lu", static_cast<unsigned long>(cur));
        return;
    }

    // Parse requested baud (accept hex or dec)
    uint32_t newBaud = parseHexOrDec(tokens[1]);
    if (newBaud == 0) {
        serial_.writeLine("ERR: invalid CAN baud");
        return;
    }

    // Apply new bus speed
    can_controller_.setBusSpeed(newBaud);
    serial_.writeLinef("Updated CAN baud rate to: %lu", static_cast<unsigned long>(can_controller_.getBusSpeed()));
}

void CANAnalyzerStack::flushBank1() {
    for (CANBusMessage &message : emulation_bank1_) {
        message.addr = 0;
        message.publish_period_ms = 0;
        message.last_published_time_ms = 0;
    }
    bank1_index_ = 0;
    serial_.writeLine("Cleared all CAN Bus emulation IDs in Bank 1");
}

void CANAnalyzerStack::flushBank2() {
    for (CANBusMessage &message : emulation_bank2_) {
        message.addr = 0;
        message.publish_period_ms = 0;
        message.last_published_time_ms = 0;
    }
    bank2_index_ = 0;
    serial_.writeLine("Cleared all CAN Bus emulation IDs in Bank 2");
}

void CANAnalyzerStack::addMsgBank1(const std::string &cmd) {
    // Tokenize like singlePacketCommand: expect 10 tokens (m addr b0..b7)
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);

    if (tokens.size() != 10 && tokens.size() != 11) {
        serial_.writeLine("ERR: expected 9 arguments after 'm' (optional period)");
        return;
    }

    uint32_t addr = parseHexOrDec(tokens[1]);
    CANBusMessage msg;
    msg.addr = addr;
    for (int i = 0; i < 8; ++i) {
        msg.bytes[i] = static_cast<uint8_t>(parseHexOrDec(tokens[2 + i]) & 0xFF);
    }

    uint32_t period = 100;
    if (tokens.size() == 11) {
        period = parseHexOrDec(tokens[10]);
    }

    addOrUpdateMsgBank(msg, emulation_bank1_, bank1_index_, "Bank 1", period);
}

void CANAnalyzerStack::addMsgBank2(const std::string &cmd) {
    // Tokenize like singlePacketCommand: expect 10 tokens (n addr b0..b7)
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);

    if (tokens.size() != 10 && tokens.size() != 11) {
        serial_.writeLine("ERR: expected 9 arguments after 'n' (optional period)");
        return;
    }

    uint32_t addr = parseHexOrDec(tokens[1]);
    CANBusMessage msg;
    msg.addr = addr;
    for (int i = 0; i < 8; ++i) {
        msg.bytes[i] = static_cast<uint8_t>(parseHexOrDec(tokens[2 + i]) & 0xFF);
    }

    uint32_t period = 100;
    if (tokens.size() == 11) {
        period = parseHexOrDec(tokens[10]);
    }

    addOrUpdateMsgBank(msg, emulation_bank2_, bank2_index_, "Bank 2", period);
}

void CANAnalyzerStack::addOrUpdateMsgBank(const CANBusMessage &msg, std::vector<CANBusMessage> &bank, uint8_t &bank_index, const char *bankName, uint32_t publish_period_ms)
{
    // Search for existing message with same addr
    for (size_t i = 0; i < bank.size(); ++i) {
        if (bank[i].addr != 0 && bank[i].addr == msg.addr) {
            // Update bytes
            for (int b = 0; b < 8; ++b) bank[i].bytes[b] = msg.bytes[b];
            // Reset timers and period
            bank[i].publish_period_ms = publish_period_ms;
            bank[i].last_published_time_ms = getSystemMillis();

            serial_.writeLinef("Updated message in %s", bankName);
            printMessageBanks();
            return;
        }
    }

    // Not found: insert at current index (circular)
    bank[bank_index] = msg;
    // Set period and reset timer for the newly added message
    bank[bank_index].publish_period_ms = publish_period_ms;
    bank[bank_index].last_published_time_ms = getSystemMillis();

    bank_index = static_cast<uint8_t>((bank_index + 1) % EMULATION_BANK_MSG_COUNT);
    serial_.writeLinef("Added message to %s", bankName);
    printMessageBanks();
}

void CANAnalyzerStack::tokenizeCommand(const std::string &cmd, std::vector<std::string> &tokens) const {
    tokens.clear();
    std::string temp;
    for (char c : cmd) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (!temp.empty()) { tokens.push_back(temp); temp.clear(); }
        } else {
            temp.push_back(c);
        }
    }
    if (!temp.empty()) tokens.push_back(temp);
}

// Print both emulation banks side-by-side.
void CANAnalyzerStack::printMessageBanks()
{
    // Prepare strings for each bank
    std::vector<std::string> left(EMULATION_BANK_MSG_COUNT), right(EMULATION_BANK_MSG_COUNT);
    auto formatMsg = [](const CANBusMessage &m)->std::string {
        if (m.addr == 0) return std::string("");
        char buf[96];
        std::snprintf(buf, sizeof(buf), "0x%03X: %02X %02X %02X %02X %02X %02X %02X %02X",
                      static_cast<unsigned int>(m.addr), m.bytes[0], m.bytes[1], m.bytes[2], m.bytes[3], m.bytes[4], m.bytes[5], m.bytes[6], m.bytes[7]);
        return std::string(buf);
    };

    size_t leftW = 0, rightW = 0;
    for (size_t i = 0; i < EMULATION_BANK_MSG_COUNT; ++i) {
        left[i] = formatMsg(emulation_bank1_[i]);
        right[i] = formatMsg(emulation_bank2_[i]);
        if (left[i].length() > leftW) leftW = left[i].length();
        if (right[i].length() > rightW) rightW = right[i].length();
    }

    // Make both columns the same width: the larger of the two (with minimum)
    size_t maxW = std::max(leftW, rightW);
    if (maxW == 0) maxW = 6;
    leftW = rightW = maxW;

    // Titles
    const std::string leftTitle = "Bank 1";
    const std::string rightTitle = "Bank 2";

    auto pad = [](const std::string &s, size_t w){
        if (s.length() >= w) return s.substr(0, w);
        return s + std::string(w - s.length(), ' ');
    };

    std::string line = "| " + pad(leftTitle, leftW) + " | " + pad(rightTitle, rightW) + " |";
    std::string border(line.size(), '=');

    serial_.writeLine(border);
    serial_.writeLine(line);
    serial_.writeLine(border);

    for (size_t i = 0; i < EMULATION_BANK_MSG_COUNT; ++i) {
        std::string l = pad(left[i], leftW);
        std::string r = pad(right[i], rightW);
        std::string row = "| " + l + " | " + r + " |";
        serial_.writeLine(row);
    }

    serial_.writeLine(border);
}

void CANAnalyzerStack::dataLoopMode(const std::string &cmd) {
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);
    if (tokens.size() < 4) {
        serial_.writeLine("ERR: expected address, mask, and delay after 'l'");
        return;
    }

    const uint32_t address = parseHexOrDec(tokens[1]);
    const uint8_t mask = static_cast<uint8_t>(parseHexOrDec(tokens[2]) & 0xFF);
    const uint32_t period = parseHexOrDec(tokens[3]);
    if (address == 0 || period == 0) {
        serial_.writeLine("ERR: loop address and delay must be non-zero");
        return;
    }

    loop_type_ = LoopType::Data;
    loop_message_.update(address, 0, 0, 0, 0, 0, 0, 0, 0);
    loop_message_.publish_period_ms = period;
    loop_message_.last_published_time_ms = getSystemMillis() - period;
    loop_address_ = address;
    loop_end_address_ = address;
    loop_address_step_ = 1;
    loop_data_mask_ = mask;
    loop_data_value_ = 0;
    serial_.writeLine("Starting data loop; press 'x' to quit");
}

void CANAnalyzerStack::addrLoopMode(const std::string &cmd) {
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);
    if (tokens.size() < 5) {
        serial_.writeLine("ERR: expected start, end, data, delay, and step after 'o'");
        return;
    }

    const uint32_t start = parseHexOrDec(tokens[1]);
    const uint32_t end = parseHexOrDec(tokens[2]);
    const uint8_t data = static_cast<uint8_t>(parseHexOrDec(tokens[3]) & 0xFF);
    const uint32_t period = parseHexOrDec(tokens[4]);
    const uint32_t step = (tokens.size() > 5) ? parseHexOrDec(tokens[5]) : 1;
    if (start == 0 || start > end || period == 0 || step == 0) {
        serial_.writeLine("ERR: invalid address loop arguments");
        return;
    }

    loop_type_ = LoopType::Address;
    loop_message_.update(start, data, data, data, data, data, data, data, data);
    loop_message_.publish_period_ms = period;
    loop_message_.last_published_time_ms = getSystemMillis() - period;
    loop_address_ = start;
    loop_end_address_ = end;
    loop_address_step_ = step;
    loop_data_mask_ = 0;
    loop_data_value_ = 0;
    serial_.writeLine("Starting address loop; press 'x' to quit");
}

void CANAnalyzerStack::comboLoopMode(const std::string &cmd) {
    std::vector<std::string> tokens;
    tokenizeCommand(cmd, tokens);
    if (tokens.size() < 5) {
        serial_.writeLine("ERR: expected start, end, delay, and step after 'k'");
        return;
    }

    const uint32_t start = parseHexOrDec(tokens[1]);
    const uint32_t end = parseHexOrDec(tokens[2]);
    const uint32_t period = parseHexOrDec(tokens[3]);
    const uint32_t step = (tokens.size() > 4) ? parseHexOrDec(tokens[4]) : 1;
    if (start == 0 || start > end || period == 0 || step == 0) {
        serial_.writeLine("ERR: invalid combo loop arguments");
        return;
    }

    loop_type_ = LoopType::Combo;
    loop_message_.update(start, 0, 0, 0, 0, 0, 0, 0, 0);
    loop_message_.publish_period_ms = period;
    loop_message_.last_published_time_ms = getSystemMillis() - period;
    loop_address_ = start;
    loop_end_address_ = end;
    loop_address_step_ = step;
    loop_data_mask_ = 0xFF;
    loop_data_value_ = 0;
    serial_.writeLine("Starting combo loop; press 'x' to quit");
}

void CANAnalyzerStack::stopLoop() {
    const bool was_active = loop_type_ != LoopType::None;
    loop_type_ = LoopType::None;
    loop_message_.publish_period_ms = 0;
    if (was_active) {
        serial_.writeLine("Stopped active loop");
    } else {
        serial_.writeLine("No active loop");
    }
}

void CANAnalyzerStack::queryMessages() const {
    serial_.writeLine("Not implemented: query messages");
}
