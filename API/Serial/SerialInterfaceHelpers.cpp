#include "SerialInterfaceHelpers.h"

void SerialInterfaceHelpers::writeLine(const char *string) {
    if (!string) {
        return;
    }

    write(const_cast<char *>(string));
    write(const_cast<char *>("\n"));
}

void SerialInterfaceHelpers::writeLine(const std::string &string) {
    writeLine(string.c_str());
}

void SerialInterfaceHelpers::writef(const char *format, ...) {
    if (!format) {
        return;
    }

    char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        return;
    }

    if (len >= (int)sizeof(buffer)) {
        len = sizeof(buffer) - 1;
        buffer[len] = '\0';
    }

    write(buffer);
}

void SerialInterfaceHelpers::writeLinef(const char *format, ...) {
    if (!format) {
        return;
    }

    char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        return;
    }

    if (len >= (int)sizeof(buffer) - 1) {
        len = sizeof(buffer) - 2;
    }

    buffer[len] = '\n';
    buffer[len + 1] = '\0';

    write(buffer);
}

std::vector<std::string> SerialInterfaceHelpers::readLines(uint32_t max_len) {
    std::vector<std::string> lines;
    if (max_len == 0) {
        return lines;
    }

    std::string buffer;
    buffer.resize(max_len);

    int bytesRead = readBytes(&buffer[0], max_len);
    if (bytesRead <= 0) {
        return lines;
    }

    buffer.resize(bytesRead);
    return splitLines(buffer.c_str(), static_cast<uint32_t>(bytesRead));
}

std::vector<std::string> SerialInterfaceHelpers::splitLines(const char *data, uint32_t len) {
    std::vector<std::string> lines;
    uint32_t start = 0;
    for (uint32_t i = 0; i < len; ++i) {
        if (data[i] == '\n') {
            uint32_t lineLen = i - start;
            if (lineLen > 0 && data[start + lineLen - 1] == '\r') {
                --lineLen;
            }

            lines.emplace_back(data + start, lineLen);
            start = i + 1;
        }
    }

    if (start < len) {
        uint32_t lineLen = len - start;
        if (lineLen > 0 && data[start + lineLen - 1] == '\r') {
            --lineLen;
        }
        lines.emplace_back(data + start, lineLen);
    }

    return lines;
}
