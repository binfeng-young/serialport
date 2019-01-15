/**
 * Created by binfeng.yang on 2018/4/8.
 *
 * Packet Formats
 *   Format:
 *    +--------------------------------+-------------------+
 *    |          header                |  data    | footer |
 *    +-------+-------+------+--------------------+--------+
 *    | sync1 | sync2 | .... | cmdID   |  .....   | CRC 16 |
 *    +-------+-------+------+--------------------+--------+
**/
#ifndef SERIALPORT_PACKET_H
#define SERIALPORT_PACKET_H

#include <cstdint>
#include "databuffer.h"
#include <cl_endian.h>

#define MAX_LENGTH                      64
#define MAX_MAP_LENGTH                  1024
#define SYNC1                           0XB5
#define SYNC2                           0X62


#define UPLOAD_MOTOR_PACKET_ID          0x30
#define UPLOAD_IO_PACKET_ID             0x31
#define UPLOAD_STATUS_PACKET_ID         0x41
#define UOLOAD_VERSION_PACKET_ID        0x51

#define UPLOAD_STATE_PACKET_ID          0X01
#define UPLOAD_MAP_PACKET_ID            0X02
#define UPLOAD_WIFI_CONTROL_PACKET_ID   0x03
#define UPLOAD_CURRENT_POSE_ID          0x04
#define UPLOAD_NAV_PATH_ID              0x05
#define UPLOAD_BOUND_ID                 0x06
#define UPLOAD_MOVE_ID                  0x07


enum SyncState {
    STATE_SYNC1,
    STATE_SYNC2,
    STATE_LENGTH,
    STATE_ACQUIRE_DATA
};
class Packet {
public:
    Packet(uint16_t bufferSize, uint16_t headerLength, uint16_t footerLength);

    ~Packet();

    void empty();

    void resetValid();

    void setSize(size_t size);

    size_t getSize();

    void dataToBuf(const void *data, int length);

    uint8_t getID();

    void setID(uint8_t id);

    uint16_t calcCheckSum();

    bool verifyCheckSum();

    void resetRead();

    void setBuf(char *buf);

    DataBuffer getBuf();

    uint16_t getDataLength();

    void setReadLength(uint16_t readLength);

    uint16_t getReadLength();

    uint16_t getHeaderLength();

    uint16_t getFooterLength();

    uint16_t getDataReadLength();

    size_t write(const void *data, size_t len);

    size_t read(void *data, size_t len);

    template<typename Type>
    void writeVal(Type val, bool big = true)
    {
        if (big) {
            Endian::swap_if_little(&val, sizeof(val));
        } else {
            Endian::swap_if_big(&val, sizeof(val));
        }
        write(static_cast<const void*>(&val), sizeof(Type));
    }

    template<typename Type>
    Type readVal(bool big = true)
    {
        Type ret;
        size_t len = sizeof(Type);
        read(static_cast<void*>(&ret), len);
        if (big) {
            Endian::swap_if_little(&ret, len);
        } else {
            Endian::swap_if_big(&ret, len);
        }
        return ret;
    }

    bool isNextGood(int bytes);

    void finalizePacket();

private:
    bool hasWriteCapacity(int bytes);
private:
    DataBuffer buffer_;
    bool m_isValid;
    uint16_t position_;
    uint16_t headerLength_;
    uint16_t footerLength_;
};

#endif //SERIALPORT_PACKET_H
