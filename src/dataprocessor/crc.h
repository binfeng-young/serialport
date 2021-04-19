//
// Created by binfeng.yang on 2021/4/14.
//

#ifndef SERIALPORT_CRC_H
#define SERIALPORT_CRC_H
#include <cstdint>
class Crc {
public:
    static uint16_t update(uint16_t crc, const uint8_t *data, int32_t length);
};

#endif // SERIALPORT_CRC_H
