//
// Created by binfeng.yang on 2018/4/8.
//

#include "packet.h"
#include <cstring>

Packet::Packet(uint16_t bufferSize, uint16_t headerLength, uint16_t footerLength)
{
    m_buf = new char[bufferSize];
    m_maxLength = bufferSize;
    m_headerLength = headerLength;
    m_footerLength = footerLength;
    m_readLength = m_headerLength;
    m_length = m_headerLength;
    m_isValid = true;
}

Packet::~Packet()
{
    if (nullptr != m_buf) {
        delete[] m_buf;
        m_buf = nullptr;
    }
}

void Packet::setBuf(char *buf)
{
    memcpy(&m_buf, buf, strlen(buf));
}

const char *Packet::getBuf()
{
    return m_buf;
}

bool Packet::setLength(uint16_t length)
{
    if (length > m_maxLength)
        return false;
    m_length = length;
    return true;
}

uint16_t Packet::getLength()
{
    return m_length;
}

uint16_t Packet::getDataLength()
{
    return m_length - m_headerLength - m_footerLength;
}

void Packet::setReadLength(uint16_t readLength)
{
    m_readLength = readLength;
}

uint16_t Packet::getReadLength()
{
    return m_readLength;
}

bool Packet::setHeaderLength(uint16_t length)
{
    if (length > m_maxLength)
        return false;
    m_headerLength = length;
    return true;
}

uint16_t Packet::getHeaderLength()
{
    return m_headerLength;
}

uint16_t Packet::getFooterLength()
{
    return m_footerLength;
}


uint16_t Packet::getDataReadLength()
{
    return m_readLength - m_headerLength;
}

void Packet::resetRead()
{
    m_readLength = m_headerLength;
    resetValid();
}

void Packet::resetValid()
{
    m_isValid = true;
}

void Packet::empty()
{
    m_length = m_headerLength;
    resetValid();
}

void Packet::byteToBuf(int8_t val)
{
    if (!hasWriteCapacity(1)) {
        return;
    }
    memcpy(m_buf + m_length, &val, 1);
    m_length += 1;
}

void Packet::byte2ToBuf(int16_t val)
{
    unsigned char c;
    if (!hasWriteCapacity(2)) {
        return;
    }
    c = (val >> 8) & 0xff;
    memcpy(m_buf + m_length, &c, 1);
    c = val & 0xff;
    memcpy(m_buf + m_length + 1, &c, 1);
    m_length += 2;
}

void Packet::byte4ToBuf(int32_t val)
{
    unsigned char c;
    if (!hasWriteCapacity(4)) {
        return;
    }
//    memcpy(m_buf + m_length, &val, 4);
//    m_length += 4;
    c = (val >> 24) & 0xff;
    memcpy(m_buf + m_length, &c, 1);
    c = (val >> 16) & 0xff;
    memcpy(m_buf + m_length + 1, &c, 1);
    c = (val >> 8) & 0xff;
    memcpy(m_buf + m_length + 2, &c, 1);
    c = val & 0xff;
    memcpy(m_buf + m_length + 3, &c, 1);
    m_length += 4;
}

void Packet::uByteToBuf(uint8_t val)
{
    if (!hasWriteCapacity(1)) {
        return;
    }
    memcpy(m_buf + m_length, &val, 1);
    m_length += 1;
}

void Packet::uByteSToBuf(uint8_t *buf, uint8_t num)
{
    for (uint8_t i = 0; i < num; i++) {
        uByteToBuf(buf[i]);
    }
}

void Packet::uByte2ToBuf(uint16_t val)
{
    unsigned char c;
    if (!hasWriteCapacity(2)) {
        return;
    }
    c = (val >> 8) & 0xff;
    memcpy(m_buf + m_length, &c, 1);
    c = val & 0xff;
    memcpy(m_buf + m_length + 1, &c, 1);
    m_length += 2;
}

void Packet::uByte4ToBuf(uint32_t val)
{
    unsigned char c;
    if (!hasWriteCapacity(4)) {
        return;
    }
//    memcpy(m_buf + m_length, &val, 4);
//    m_length += 4;
    c = (val >> 24) & 0xff;
    memcpy(m_buf + m_length, &c, 1);
    c = (val >> 16) & 0xff;
    memcpy(m_buf + m_length + 1, &c, 1);
    c = (val >> 8) & 0xff;
    memcpy(m_buf + m_length + 2, &c, 1);
    c = val & 0xff;
    memcpy(m_buf + m_length + 3, &c, 1);
    m_length += 4;
}

void Packet::strToBuf(const char *str)
{
    uint16_t tempLen;
    if (str == NULL) {
        str = "";
    }
    tempLen = strlen(str) + 1;

    if (!hasWriteCapacity(tempLen)) {
        return;
    }
    memcpy(m_buf + m_length, str, tempLen);
    m_length += tempLen;
}

void Packet::strNToBuf(const char *str, int length)
{
    // Do not perform bounds checking because it breaks existing code.

    //byte4ToBuf(length);
    memcpy(m_buf + m_length, str, length);
    m_length += length;
}

void Packet::strToBufPadded(const char *str, int length)
{
    uint16_t tempLen;
    if (str == NULL) {
        str = "";
    }
    tempLen = strlen(str);
    if (!hasWriteCapacity(length)) {
        return;
    }

    if (tempLen >= length) {
        memcpy(m_buf + m_length, str, length);
        m_length += length;
    } else     // string is smaller than given length
    {
        memcpy(m_buf + m_length, str, tempLen);
        m_length += tempLen;
        memset(m_buf + m_length, 0, length - tempLen);
        m_length += length - tempLen;
    }
}

void Packet::dataToBuf(const void *data, int length)
{
    if (data == NULL) {
        return;
    }

    if (!hasWriteCapacity(length)) {
        return;
    }
    memcpy(m_buf + m_length, data, length);
    m_length += length;
}

int8_t Packet::bufToByte()
{
    int8_t ret = 0;

    if (isNextGood(1)) {
        memcpy(&ret, m_buf + m_readLength, 1);
        m_readLength += 1;
    }
    return (ret);
}

int16_t Packet::bufToByte2()
{
    int16_t ret = 0;
    unsigned char c1, c2;

    if (isNextGood(2)) {
        memcpy(&c1, m_buf + m_readLength + 1, 1);
        memcpy(&c2, m_buf + m_readLength, 1);
        ret = (c1 & 0xff) | (c2 << 8);
        m_readLength += 2;
    }
    return ret;
}

int32_t Packet::bufToByte4()
{
    int32_t ret = 0;
    unsigned char c1, c2, c3, c4;

    if (isNextGood(4)) {
        memcpy(&c1, m_buf + m_readLength + 3, 1);
        memcpy(&c2, m_buf + m_readLength + 2, 1);
        memcpy(&c3, m_buf + m_readLength + 1, 1);
        memcpy(&c4, m_buf + m_readLength, 1);
        ret = (c1 & 0xff) | (c2 << 8) | (c3 << 16) | (c4 << 24);
        m_readLength += 4;
    }
    return ret;
}

uint8_t Packet::bufToUByte()
{
    uint8_t ret = 0;

    if (isNextGood(1)) {
        memcpy(&ret, m_buf + m_readLength, 1);
        m_readLength += 1;
    }
    return (ret);
}

void Packet::bufToUByteS(uint8_t *buf, uint8_t num)
{

    for (uint8_t i = 0; i < num; i++) {
        buf[i] = bufToUByte();
    }

}

uint16_t Packet::bufToUByte2()
{
    uint16_t ret = 0;
    unsigned char c1, c2;

    if (isNextGood(2)) {
        memcpy(&c1, m_buf + m_readLength + 1, 1);
        memcpy(&c2, m_buf + m_readLength, 1);
        ret = (c1 & 0xff) | (c2 << 8);
        m_readLength += 2;
    }

    return ret;
}

uint32_t Packet::bufToUByte4()
{
    int32_t ret = 0;
    unsigned char c1, c2, c3, c4;

    if (isNextGood(4)) {
        memcpy(&c1, m_buf + m_readLength + 3, 1);
        memcpy(&c2, m_buf + m_readLength + 2, 1);
        memcpy(&c3, m_buf + m_readLength + 1, 1);
        memcpy(&c4, m_buf + m_readLength, 1);
        ret = (c1 & 0xff) | (c2 << 8) | (c3 << 16) | (c4 << 24);
        m_readLength += 4;
    }

    return ret;
}

void Packet::bufToStr(char *buf, int len)
{
    int i;
    if (buf == NULL) {
        return;
    }

    buf[0] = '\0';
    // see if we can read
    if (isNextGood(1)) {
        // while we can read copy over those bytes
        for (i = 0; isNextGood(1) && i < (len - 1) && m_buf[m_readLength] != '\0';
             ++m_readLength, ++i) {
            buf[i] = m_buf[m_readLength];
        }
        // if we stopped because of a null then copy that one too
        if (m_buf[m_readLength] == '\0') {
            buf[i] = m_buf[m_readLength];
            m_readLength++;
        } else if (i >= (len - 1)) {

            // Otherwise, if we stopped reading because the output buffer was full,
            // then attempt to flush the rest of the string from the packet

            // This is a bit redundant with the code below, but wanted to log the
            // string for debugging
            m_buf[len - 1] = '\0';

            while (isNextGood(1) && (m_buf[m_readLength] != '\0')) {
                m_readLength++;
            }
            if (m_buf[m_readLength] == '\0') {
                m_readLength++;
            }
        } // end else if output buffer filled before null-terminator
    } // end if something to read

    // Make absolutely sure that the string is null-terminated...
    buf[len - 1] = '\0';
}

void Packet::bufToData(char *data, int length)
{
    if (data == NULL) {
        return;
    }
    if (isNextGood(length)) {
        memcpy(data, m_buf + m_readLength, length);
        m_readLength += length;
    }
}

//void duplicatePacket(Packet_t *src)
//{
//    m_length = src->length;
//    m_readLength = m_readLength;
//    memcpy(m_buf, m_buf, m_length);
//}

bool Packet::isNextGood(int bytes)
{
    if (bytes <= 0)
        return false;

    // make sure it comes in before the header
    if (m_readLength + bytes <= m_length - m_footerLength)
        return true;

    m_isValid = false;
    return false;
}

bool Packet::hasWriteCapacity(int bytes)
{
    if (bytes < 0) {
        return false;
    }

    // Make sure there's enough room in the packet
    if ((m_length + bytes) <= m_maxLength) {
        return true;
    }

    m_isValid = false;

    return false;

} // end method hasWriteCapacity

bool Packet::isValid()
{
    return m_isValid;
} // end method isValid

uint8_t Packet::getID()
{
    if (m_length >= 5)
        return m_buf[4];
    else
        return 0;
}


void Packet::setID(uint8_t id)
{
    m_buf[4] = id;
}

bool Packet::verifyCheckSum()
{
    unsigned short chksum;
    unsigned char c1, c2;

    if (m_length - 2 < m_headerLength)
        return false;
    c2 = m_buf[m_length - 2];
    c1 = m_buf[m_length - 1];
    chksum = (c1 & 0xff) | (c2 << 8);

    if (chksum == calcCheckSum()) {
        return true;
    } else {
        return false;
    }
}

uint16_t Packet::calcCheckSum()
{
    int i;
    unsigned char n;
    unsigned short c = 0;

    i = 4;
    n = ((m_buf[2] << 8) | m_buf[3]) - 2;
    while (n > 1) {
        c += ((unsigned char) m_buf[i] << 8) | (unsigned char) m_buf[i + 1];
        c = c & 0xffff;
        n -= 2;
        i += 2;
    }
    if (n > 0)
        c = c ^ (unsigned short) ((unsigned char) m_buf[i]);
    return c;
}

void Packet::finalizePacket()
{
    uint16_t len = m_length;
    uint16_t chkSum;

    m_length = 0;
    uByteToBuf((uint8_t) SYNC1);
    uByteToBuf((uint8_t) SYNC2);
    uByte2ToBuf(len - m_headerLength + 3);
    m_length = len;

    chkSum = calcCheckSum();
    uByte2ToBuf(chkSum);
}

//
//Packet_t copyPacket()
//{
//    Packet_t temp;
//    temp.footerLength = m_footerLength;
//    temp.headerLength = m_headerLength;
//    temp.isValid = m_isValid;
//    temp.length = m_length;
//    temp.readLength = m_readLength;
//    memcpy(&temp.buf, &m_buf, m_length);
//    return temp;
//}


