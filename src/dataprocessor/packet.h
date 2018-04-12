//
// Created by binfeng.yang on 2018/4/8.
//
#ifndef SERIALPORT_PACKET_H
#define SERIALPORT_PACKET_H

#include <cstdint>

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


enum SyncState {
    STATE_SYNC1,
    STATE_SYNC2,
    STATE_LENGTH,
    STATE_ACQUIRE_DATA
};

enum Packets {
    MOTOR = 0X30,
    IO_SENSORS = 0x31
};
class Packet {
public:
    Packet(uint16_t bufferSize, uint16_t headerLength, uint16_t footerLength);

    ~Packet();

    void empty();

    void resetValid();

    bool setLength(uint16_t length);

    void uByteToBuf(uint8_t val);

    void uByte2ToBuf(uint16_t val);

    void dataToBuf(const void *data, int length);

    uint8_t getID();

    void setID(uint8_t id);

    uint16_t calcCheckSum();

    bool verifyCheckSum();

    void resetRead();

    void create(uint16_t bufferSize, uint16_t headerLength, char *buf, uint16_t footerLength);

    void setBuf(char *buf);

    const char *getBuf();

    void setMaxLength(uint16_t bufferSize);

    uint16_t getMaxLength();


    uint16_t getLength();

    uint16_t getDataLength();

    void setReadLength(uint16_t readLength);

    uint16_t getReadLength();

    bool setHeaderLength(uint16_t length);

    uint16_t getHeaderLength();

    bool setFooterLength(uint16_t length);

    uint16_t getFooterLength();

    uint16_t getDataReadLength();


    void byteToBuf(int8_t val);

    void byte2ToBuf(int16_t val);

    void byte4ToBuf(int32_t val);


    void uByteSToBuf(uint8_t *buf, uint8_t num);

    void uByte4ToBuf(uint32_t val);

/**
  @param str string to copy into buffer
*/
    void strToBuf(const char *str);

    void strNToBuf(const char *str, int length);

    void strToBufPadded(const char *str, int length);

// void dataToBuf(const unsigned char *data, int length);

    int8_t bufToByte();

    int16_t bufToByte2();

    int32_t bufToByte4();

    uint8_t bufToUByte();

    void bufToUByteS(uint8_t *buf, uint8_t num);

    uint16_t bufToUByte2();

    uint32_t bufToUByte4();

    void bufToStr(char *buf, int len);

    void bufToData(char *data, int length);

    //void duplicatePacket(Packet_t *src, Packet_t *dist);

    bool isNextGood(int bytes);

    bool isValid();

    //void duplicatePacket();

    void finalizePacket();

private:
    bool hasWriteCapacity(int bytes);

private:
    char *m_buf;
    bool m_isValid;
    uint16_t m_readLength;
    uint16_t m_length;
    uint16_t m_maxLength;
    uint16_t m_headerLength;
    uint16_t m_footerLength;
};

#endif //SERIALPORT_PACKET_H
