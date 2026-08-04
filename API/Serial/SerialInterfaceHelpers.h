#pragma once

#include "ISerialInterface.h"

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

/// @brief Helper base class for serial implementations.
/// Uses the abstract ISerialInterface methods for actual I/O.
class SerialInterfaceHelpers : public ISerialInterface {
    public:
    SerialInterfaceHelpers() = default;
    virtual ~SerialInterfaceHelpers() = default;

    /// @brief Writes a string and appends a newline.
    void writeLine(const char *string);
    void writeLine(const std::string &string);

    /// @brief Writes a formatted string using varargs.
    void writef(const char *format, ...);
    void writeLinef(const char *format, ...);

    /// @brief Reads bytes and splits the result on newline boundaries.
    /// @param max_len Maximum number of bytes to read.
    /// @return Vector of lines read from the receive buffer.
    std::vector<std::string> readLines(uint32_t max_len);

    protected:
    static std::vector<std::string> splitLines(const char *data, uint32_t len);
};
