//
// Created by binfeng.yang on 2018/4/8.
//

#include "packet.h"
#include <cstring>

Packet::Packet(uint16_t bufferSize, uint16_t headerLength, uint16_t footerLength)
{
    buffer_.set_capacity(bufferSize);
    headerLength_ = headerLength;
    footerLength_ = footerLength;
    position_ = headerLength_;
    buffer_.set_size(headerLength_);
    m_isValid = true;
}

Packet::~Packet() {}

void Packet::setBuf(char *buf)
{
    memcpy(buffer_.get_data(), buf, strlen(buf));
}

DataBuffer Packet::getBuf()
{
    return buffer_;
}

void Packet::setSize(size_t size)
{
    buffer_.set_size(size);
}

size_t Packet::getSize()
{
    return buffer_.get_size();
}

uint16_t Packet::getDataLength()
{
    return buffer_.get_size() - headerLength_ - footerLength_;
}

void Packet::setReadLength(uint16_t readLength)
{
    position_ = readLength;
}

uint16_t Packet::getReadLength()
{
    return position_;
}

uint16_t Packet::getHeaderLength()
{
    return headerLength_;
}

uint16_t Packet::getFooterLength()
{
    return footerLength_;
}


uint16_t Packet::getDataReadLength()
{
    return position_ - headerLength_;
}

void Packet::resetRead()
{
    position_ = headerLength_;
    resetValid();
}

void Packet::resetValid()
{
    m_isValid = true;
}

void Packet::empty()
{
    setSize(headerLength_);
    resetValid();
}
size_t Packet::write(const void *data, size_t len)
{
    if (!hasWriteCapacity(len)) {
        return 0;
    }
    size_t size = buffer_.get_size();
    setSize(size + len);
    memcpy(buffer_.get_data() + size, data, len);
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
    memcpy(data, buffer_.get_data() + position_, len);
    position_ += len;
    return len;
}

bool Packet::isNextGood(int bytes)
{
    if (bytes <= 0)
        return false;

    // make sure it comes in before the header
    if (position_ + bytes <= (int)buffer_.get_size() - footerLength_)
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
    if ((buffer_.get_size() + bytes) <= buffer_.get_capacity()) {
        return true;
    }

    m_isValid = false;

    return false;

} // end method hasWriteCapacity

uint8_t Packet::getID()
{
    if (buffer_.get_size() >= headerLength_)
        return static_cast<uint8_t>(buffer_[headerLength_ - 1]);
    else
        return 0;
}


void Packet::setID(uint8_t id)
{
    buffer_[headerLength_ - 1] = id;
}

bool Packet::verifyCheckSum()
{
    size_t size = buffer_.get_size();
    if (size - 2 < headerLength_)
        return false;
    auto c2 = static_cast<unsigned char>(buffer_[size - 2]);
    auto c1 = static_cast<unsigned char>(buffer_[size - 1]);
    auto check_sum = static_cast<uint16_t >((c1 & 0xff) | (c2 << 8));

    return check_sum == calcCheckSum();
}

uint16_t Packet::calcCheckSum()
{
    unsigned short c = 0;

    int i = 4;
    unsigned char n = (((unsigned char)buffer_[2] << 8) | ((unsigned char)buffer_[3])) - 2;
    while (n > 1) {
        c += ((unsigned char) buffer_[i] << 8) | (unsigned char) buffer_[i + 1];
        c = c & 0xffff;
        n -= 2;
        i += 2;
    }
    if (n > 0)
        c = c ^ (unsigned short) ((unsigned char) buffer_[i]);
    return c;
}

void Packet::finalizePacket()
{
    uint16_t len = buffer_.get_size();
    uint16_t chkSum;

    buffer_.set_size(0);
    writeVal<uint8_t>((uint8_t) SYNC1);
    writeVal<uint8_t>((uint8_t) SYNC2);
    writeVal<uint16_t>(len - headerLength_ + 4);
    buffer_.set_size(len);

    chkSum = calcCheckSum();
    writeVal<uint16_t>(chkSum);
}

