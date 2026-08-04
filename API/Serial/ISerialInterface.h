#pragma once

#include <stdint.h>
#include <string>
#include <vector>

/// @brief Platform-agnostic interface for text-based serial I/O
class ISerialInterface {
    public:
    virtual ~ISerialInterface() = default;

    /// @brief Write a null-terminated string to the serial interface.
    /// @param string Zero-terminated ASCII string to send.
    virtual void write(char *string) = 0;

    /// @brief Read a single byte from the receive buffer.
    /// @return Received byte, or 0 if no data is available.
    virtual char read() = 0;

    /// @brief Read up to max_len bytes into buf.
    /// @param buf Destination buffer.
    /// @param max_len Maximum number of bytes to read.
    /// @return Number of bytes read, or -1 on error.
    virtual int readBytes(char *buf, uint32_t max_len) = 0;

    /// @brief Check if data is available to read.
    /// @return true if at least one byte is available.
    virtual bool available() = 0;

    /// @brief Attempts to read bytes from the serial buffer and splits them into strings separated by '\n' or '\r'
    /// @return true if at least one command is available
    virtual bool processLines(std::vector<std::string> &out_lines) = 0;
};
