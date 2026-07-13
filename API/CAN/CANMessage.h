#pragma once

#include <stdint.h>

/// @brief Generic CAN bus message with address and data fields.
class CANMessage{
  public:

    //CAN bus address of this message, standard or extended
    uint32_t addr = 0;        

    // Internal data for the CAN frame
    uint8_t bytes[8] = {0};

    // Updates the internally stored bytes from an array. Assumes the bytes array is count long
    void update(uint32_t addr, uint8_t *bytes, uint8_t count)
    {
      this->addr = addr;
      // copy up to 8 bytes; if count < 8 zero the remainder
      for(uint8_t i = 0; i < 8; ++i){
        this->bytes[i] = (i < count) ? bytes[i] : 0;
      }

    }

    // Updates the internally stored bytes from individually-passed arguments
    void update(uint32_t addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
    {
      this->addr = addr;
      this->bytes[0] = data0;
      this->bytes[1] = data1;
      this->bytes[2] = data2;
      this->bytes[3] = data3;
      this->bytes[4] = data4;
      this->bytes[5] = data5;
      this->bytes[6] = data6;
      this->bytes[7] = data7;
    }
};