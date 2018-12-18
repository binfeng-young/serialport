//
// Created by binfeng.yang on 2018/4/8.
//

#include "packet.h"
#include <cstring>

Packet::Packet(uint16_t bufferSize, uint16_t headerLength, uint16_t footerLength)
{
    buffer.set_capacity(bufferSize);
    m_headerLength = headerLength;
    m_footerLength = footerLength;
    position_ = m_headerLength;
    buffer.set_size(m_headerLength);
    m_isValid = true;
}

Packet::~Packet() {}

void Packet::setBuf(char *buf)
{
    memcpy(buffer.get_data(), buf, strlen(buf));
}

DataBuffer Packet::getBuf()
{
    return buffer;
}

void Packet::setSize(size_t size)
{
    buffer.set_size(size);
}

size_t Packet::getSize()
{
    return buffer.get_size();
}

uint16_t Packet::getDataLength()
{
    return buffer.get_size() - m_headerLength - m_footerLength;
}

void Packet::setReadLength(uint16_t readLength)
{
    position_ = readLength;
}

uint16_t Packet::getReadLength()
{
    return position_;
}

bool Packet::setHeaderLength(uint16_t length)
{
    if (length > buffer.get_capacity())
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
    return position_ - m_headerLength;
}

void Packet::resetRead()
{
    position_ = m_headerLength;
    resetValid();
}

void Packet::resetValid()
{
    m_isValid = true;
}

void Packet::empty()
{
    setSize(m_headerLength);
    resetValid();
}
size_t Packet::write(const void *data, size_t len)
{
    if (!hasWriteCapacity(len)) {
        return 0;
    }
    size_t size = buffer.get_size();
    setSize(size + len);
    memcpy(buffer.get_data() + size, data, len);
    return len;
}

void Packet::dataToBuf(const void *data, int length)
{
    if (data == NULL) {
        return;
    }
    write(data, length);
}

size_t Packet::read(void *data, size_t len)
{
    if (!isNextGood(len)) {
        return 0;
    }
    memcpy(data, buffer.get_data() + position_, len);
    position_ += len;
    return len;
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
        for (i = 0; isNextGood(1) && i < (len - 1) && buffer[position_] != '\0';
             ++position_, ++i) {
            buf[i] = buffer[position_];
        }
        // if we stopped because of a null then copy that one too
        if (buffer[position_] == '\0') {
            buf[i] = buffer[position_];
            position_++;
        } else if (i >= (len - 1)) {

            // Otherwise, if we stopped reading because the output buffer was full,
            // then attempt to flush the rest of the string from the packet

            // This is a bit redundant with the code below, but wanted to log the
            // string for debugging
            buffer[len - 1] = '\0';

            while (isNextGood(1) && (buffer[position_] != '\0')) {
                position_++;
            }
            if (buffer[position_] == '\0') {
                position_++;
            }
        } // end else if output buffer filled before null-terminator
    } // end if something to read

    // Make absolutely sure that the string is null-terminated...
    buf[len - 1] = '\0';
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
    if (position_ + bytes <= (int)buffer.get_size() - m_footerLength)
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
    if ((buffer.get_size() + bytes) <= buffer.get_capacity()) {
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
    if (buffer.get_size() >= 5)
        return static_cast<uint8_t>(buffer[4]);
    else
        return 0;
}


void Packet::setID(uint8_t id)
{
    buffer[4] = id;
}

bool Packet::verifyCheckSum()
{
    unsigned short chksum;
    unsigned char c1, c2;

    size_t size = buffer.get_size();
    if (size - 2 < m_headerLength)
        return false;
    c2 = static_cast<unsigned char>(buffer[size - 2]);
    c1 = static_cast<unsigned char>(buffer[size - 1]);
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
    unsigned char n, tmp1, tmp2;
    unsigned short c = 0;

    i = 4;
    n = (((unsigned char)buffer[2] << 8) | ((unsigned char)buffer[3])) - 2;
    while (n > 1) {
        c += ((unsigned char) buffer[i] << 8) | (unsigned char) buffer[i + 1];
        c = c & 0xffff;
        n -= 2;
        i += 2;
    }
    if (n > 0)
        c = c ^ (unsigned short) ((unsigned char) buffer[i]);
    return c;
}

void Packet::finalizePacket()
{
    uint16_t len = buffer.get_size();
    uint16_t chkSum;

    buffer.set_size(0);
    writeVal<uint8_t>((uint8_t) SYNC1);
    writeVal<uint8_t>((uint8_t) SYNC2);
    writeVal<uint16_t>(len - m_headerLength + 3);
    buffer.set_size(len);

    chkSum = calcCheckSum();
    writeVal<uint16_t>(chkSum);
}

