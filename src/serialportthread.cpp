//
// Created by binfeng.yang on 2018/4/7.
//

#include "serialportthread.h"
#include "packet.h"
#include <QSerialPort>
#include <iostream>
#include <QTime>

SerialPortThread::SerialPortThread()
    : m_serialDevice(nullptr)
      , m_portStatus(PortStatus::closed)
      , m_receiveHex(false)
{
    m_recv_packet = new Packet(64, 5, 2);
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
    reSetMap();
}

SerialPortThread::~SerialPortThread()
{
    if (nullptr != m_recv_packet) {
        delete m_recv_packet;
        m_recv_packet = nullptr;
    }
    close();
}

void SerialPortThread::open(const QString &deviceName)
{
    m_portStatus = PortStatus::error;
    m_serialDevice = new QSerialPort(deviceName);
    if (m_serialDevice->open(QIODevice::ReadWrite)) {
        if (m_serialDevice->setBaudRate(QSerialPort::Baud115200)
            && m_serialDevice->setDataBits(QSerialPort::Data8)
            && m_serialDevice->setParity(QSerialPort::NoParity)
            && m_serialDevice->setStopBits(QSerialPort::OneStop)
            && m_serialDevice->setFlowControl(QSerialPort::NoFlowControl)) {
            m_portStatus = PortStatus::open;
        }
    }
    if (m_portStatus == PortStatus::open) {
        emit opened(true);
        m_serialDevice->clear();
        start();
        connect(m_serialDevice, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
                this, &SerialPortThread::handleSerialError);
        std::cout << "open" << std::endl;
    } else {
        emit opened(false);
        std::cout << "error" << std::endl;
//        ui->sendButton->setDisabled(true);
    }
}

void SerialPortThread::close()
{
    if (nullptr != m_serialDevice) {
        //disconnect(m_serialDevice, SIGNAL(readyRead()), this, SLOT(onRead()));
        m_portStatus = PortStatus::closed;
        wait();
        m_serialDevice->deleteLater();
        m_serialDevice = nullptr;
        emit opened(false);
        std::cout << "closed" << std::endl;
    }
}


void SerialPortThread::onOpen()
{
    switch (m_portStatus) {
        case PortStatus::closed:
        case PortStatus::error:
            open(m_deviceName);
            break;
        case PortStatus::open:
            close();
            break;
    }
}

//void SerialPortThread::onSend()
//{
//    uint8_t toBLCmd[] = {0xb5, 0x62, 0x00, 0x03, 0x4c, 0x00, 0x4c};
//    if (m_serialDevice->write(reinterpret_cast<char *>(toBLCmd), 7) <= 0) {
//        std::cout << "Could not write serial" << std::endl;
//        return;
//    }
//}
//
//void SerialPortThread::onRead()
//{
//    //QByteArray arr = m_serialDevice->readAll();
//    //ui->receiveTextEdit->append(arr);
//}
bool SerialPortThread::readChar(char *c)
{
    m_serialDevice->waitForBytesWritten(1);
    if (m_serialDevice->bytesAvailable() || m_serialDevice->waitForReadyRead(10)) {
        m_serialDevice->read(c, 1);
        return true;
    } else {
        msleep(100);
        return false;
    }

}

bool SerialPortThread::readBuff(void *data, int len)
{
    QTime timer;
    timer.start();
    for (int i = 0; i < len;) {
        if (PortStatus::closed == m_portStatus) {
            return false;
        }
        char c;
        if (readChar(&c)) {
            *((char *) data + i) = c;
            timer.start();
            i++;
        } else if (timer.elapsed() > 3000) {
            std::cout << "time out" << std::endl;
            return false;
        }
    }
    return true;
}

void SerialPortThread::run()
{
    QString buff;
    bool isCmd = false;
    SyncState state = STATE_SYNC1;
    uint16_t length = 1;
    uint8_t c[64];
    while (PortStatus::open == m_portStatus) {
        uint16_t buffLength = length;
        switch (state) {
            case STATE_SYNC1:
                if (readBuff(c, length)) {
                    if (c[0] == SYNC1) {
                        //std::cout << "SYNC1" << std::endl;
                        m_recv_packet->empty();
                        m_recv_packet->setLength(0);
                        m_recv_packet->uByteToBuf(c[0]);
                        state = STATE_SYNC2;
                    }
                } else {
                    buffLength = 0;
                }
                if (c[0] == 0xaa) buff.append("\n");
                if (state == STATE_SYNC1 && c[0] != SYNC1) {
                    QString temp;
                    if (m_receiveHex) {
                        for (int i = 0; i < buffLength; i++) {
                            QByteArray hex = QByteArray((char *) &c[0], 1).toHex();
                            temp.append(hex).append(" ");
                        }
                    } else {
                        temp.append(QByteArray((char *) c, buffLength));
                    }
                    buff.append(temp);
                    if (buff.length() >= 10 || c[0] == '\n') {
                        emit showString(buff);
                        buff.clear();
                    }
                }
                break;
            case STATE_SYNC2:
                // 为帧头2时，跳转到状态3，获取数据，否则跳回状态1
                if (readBuff(c, length)) {
                    if (c[0] == SYNC2) {
                        //std::cout << "SYNC2" << std::endl;
                        state = STATE_LENGTH;
                        m_recv_packet->uByteToBuf(c[0]);
                        length = 2;
                    } else {
                        state = STATE_SYNC1;
                    }
                } else {
                    buffLength = 0;
                }
                break;
            case STATE_LENGTH:
                if (readBuff(c, length)) {
                    length = ((uint16_t) c[0] & 0xff) << 8 | ((uint16_t) c[1] & 0xff);
                    if (length > 5) {
                        //std::cout << "SYNC_LENGTH" << std::endl;
                        state = STATE_ACQUIRE_DATA;
                        m_recv_packet->uByte2ToBuf(length);
                    } else {
                        length = 1;
                        state = STATE_SYNC1;
                    }
                } else {
                    buffLength = 0;
                }
                break;
            case STATE_ACQUIRE_DATA:
                if (readBuff(c, length)) {
                    m_recv_packet->dataToBuf(c, length);
                    if (m_recv_packet->verifyCheckSum()) {
                        //std::cout << "STATE_ACQUIRE_DATA" << std::endl;
                        m_recv_packet->resetRead();
                        parseCommand();
                    }
                    state = STATE_SYNC1;
                } else {
                    buffLength = 0;
                }
                length = 1;
                break;
        }

        QThread::msleep(1);
    }
}

void SerialPortThread::deviceChanged(const QString &deviceName)
{
    m_deviceName = deviceName;
}

void SerialPortThread::onReceiveHex(int checkState)
{
    m_receiveHex = checkState == 2;
    std::cout << m_receiveHex << std::endl;
}

struct PoseData {
    uint16_t x;
    uint16_t y;
    uint8_t theta;
};
bool equalsP(const PoseData& p, const PoseData& q) {
    return p.x == q.x && p.y ==  q.y;
}

struct MapPoseData {
    uint16_t id;
    uint8_t type;
    PoseData pose;
};

struct MapData {
    uint8_t count;
    MapPoseData mapPoseData[100];
};
static PoseData lastPose = {0, 0, 0};
static bool first = true;
void SerialPortThread::reSetMap()
{
    lastPose = {0, 0, 0};
    first = true;
}
void SerialPortThread::parseCommand()
{
    switch (m_recv_packet->getID()) {
        case UPLOAD_MAP_PACKET_ID: {
            uint8_t count = m_recv_packet->bufToUByte();
            for (int i = 0; i < count; i++) {
                uint16_t id = m_recv_packet->bufToUByte2();
                uint8_t type = m_recv_packet->bufToUByte();
                uint16_t x = m_recv_packet->bufToUByte2();
                uint16_t y = m_recv_packet->bufToUByte2();
                uint8_t theta = m_recv_packet->bufToUByte();
                emit drawPoseData(x, y, theta, type);
                std::cout << id << " " << (int) type << " " << x << " " << y << " "
                          << (int) theta << std::endl;
            }
            break;
        }
        case UPLOAD_CURRENT_POSE_ID: {
            PoseData currentPose;
            currentPose.x = m_recv_packet->bufToUByte2();
            currentPose.y = m_recv_packet->bufToUByte2();
            currentPose.theta = m_recv_packet->bufToUByte();
            if (!equalsP(lastPose, currentPose)) {
                if (!first) {
                    emit drawPoseData(lastPose.x, lastPose.y, lastPose.theta, 0);
                    emit drawPath(lastPose.x, lastPose.y, currentPose.x, currentPose.y, 0);
                } else {
                    first = false;
                }
                lastPose.x = currentPose.x;
                lastPose.y = currentPose.y;
                lastPose.theta = currentPose.theta;
            }
            emit drawPoseData(currentPose.x, currentPose.y, currentPose.theta, 2);
            break;
        }
        case UPLOAD_NAV_PATH_ID : {
            std::cout << "get path ***********" << std::endl;
            PoseData prePose;
            uint8_t count = m_recv_packet->bufToUByte();
            prePose.x = m_recv_packet->bufToUByte2();
            prePose.y = m_recv_packet->bufToUByte2();
            for (int i = 1; i < count; i++) {
                PoseData currentPose;
                currentPose.x = m_recv_packet->bufToUByte2();
                currentPose.y = m_recv_packet->bufToUByte2();
                emit drawPath(prePose.x, prePose.y, currentPose.x, currentPose.y, 1);
                prePose.x = currentPose.x;
                prePose.y = currentPose.y;
            }
        }
        default:
            break;
    }
}

void SerialPortThread::handleSerialError(QSerialPort::SerialPortError error)
{

    if (error != QSerialPort::ReadError && error != QSerialPort::TimeoutError) {
        std::cout << error << std::endl;
        close();
    }
}