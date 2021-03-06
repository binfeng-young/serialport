//
// Created by binfeng.yang on 2018/4/7.
//

#include "serialportthread.h"
#include "packet.h"
#include "sensor_data.h"
#include <QSerialPort>
#include <QTime>
#include <iostream>

SerialPortThread::SerialPortThread()
    : m_serialDevice(nullptr)
      , m_portStatus(PortStatus::closed)
      , m_receiveHex(false)
{
    m_recvPacket = new Packet(1024, 8, 2);
    m_sendBuf = new char[1024]();
    m_sendPending = false;
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
    reSetMap();
}

SerialPortThread::~SerialPortThread()
{
    if (nullptr != m_recvPacket) {
        delete m_recvPacket;
        m_recvPacket = nullptr;
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
        m_queueOffSet = 0;
        m_showStringOutTime.start();
        //m_serialDevice->setDataTerminalReady(true);
        //connect(m_serialDevice, SIGNAL(readyRead()), this, SLOT(readFromSerial()));
        m_serialDevice->moveToThread(this);
        connect(m_serialDevice, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
                this, &SerialPortThread::handleSerialError);

        start();

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
        //disconnect(m_serialDevice, SIGNAL(readyRead()), this, SLOT(readFromSerial()));
        m_portStatus = PortStatus::closed;
        wait();
        delete m_serialDevice;
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

bool SerialPortThread:: writChar(char &c)
{
    m_serialDevice->write(&c, 1);
    m_serialDevice->waitForBytesWritten(1);
    return true;
}

bool SerialPortThread::send()
{
    /*for (int i = 0; i < m_sendLen; i++) {
        writChar(m_sendBuf[i]);
    }*/

    if (!m_writeArray.isEmpty()) {
        std::lock_guard<std::mutex> lock(sendbufmutex);
        std::cout << "send" << std::endl;
        m_serialDevice->write(m_writeArray);
        m_writeArray.clear();
        return true;
    }
    return false;
}
void SerialPortThread::sendBuff(const QByteArray& text)
{
    std::lock_guard<std::mutex> lock(sendbufmutex);
    m_writeArray.append(text);
/*    m_sendPending = true;
    m_sendLen = len;
    memcpy(m_sendBuf, data, len);*/
    //sendbufmutex.unlock();
}

void SerialPortThread::onSend(const QByteArray& text)
{
    if (PortStatus::closed == m_portStatus) {
        return;
    }
    //aa 55 a5 c3 34 00 00 00 00 9b
    //uint8_t toFold[] = {0xaa, 0x55, 0xa5, 0xc3, 0x34, 0, 0, 0, 0, 0x9b};
    //sendBuff(toFold, 10);
//    uint8_t virtualWall[] = {0xaa, 0x55, 0xa3, 0x27, 0x10, 0, 0, 0x27, 0x10, 0, 0, 0xf0, 0xd8, 0xff, 0xff, 0xf0, 0xd8, 0xff, 0xff, 02, 01, 0,0,0, 0x9f};
//    sendBuff(virtualWall, 25);
    sendBuff(text);
}

void SerialPortThread::readFromSerial()
{
    QByteArray arr = m_serialDevice->readAll();
    //m_readQueue.push(arr);
    std::cout << "read  "  << arr.size() << std::endl;
}
bool SerialPortThread::readChar(char& c)
{
    if (m_readArray.isEmpty()) {
        m_serialDevice->waitForBytesWritten(1);
        if (m_serialDevice->bytesAvailable() || m_serialDevice->waitForReadyRead(1)) {
            m_readArray = m_serialDevice->readAll();
            //m_readQueue.push(arr);
            //m_serialDevice->read(&c, 1);
            //return true;
            //std::cout << "read: " <<  m_readArray.size() << std::endl;
            if (m_readArray.isEmpty()) {
                return false;
            }
        } else {
            msleep(10);
            return false;
        }
    }
    //m_serialDevice->waitForReadyRead(1);
//    if (m_readQueue.empty()) {
//        return false;
//    }
//    QByteArray arr = m_readQueue.front();
//    while (arr.isEmpty()) {
//        m_readQueue.pop();
//        m_queueOffSet = 0;
//        if (m_readQueue.empty()) {
//            return false;
//        }
//    }
//
    c = m_readArray.at(m_queueOffSet++);
//
    if (m_queueOffSet >= m_readArray.size()) {
        m_queueOffSet = 0;
        //m_readQueue.pop();
        m_readArray.clear();
    }
    return true;
}

bool SerialPortThread::readBuff(void *data, int len, int timeOut)
{
    QTime timer;
    timer.start();
    for (int i = 0; i < len;) {
        if (PortStatus::closed == m_portStatus) {
            return false;
        }
        char c;
        if (readChar(c)) {
            *((char *) data + i) = c;
            timer.start();
            i++;
            //std::cout << "read  " << (int)(c) << std::endl;
        } else if (timer.elapsed() < timeOut) {
            QThread::msleep(1);
            //std::cout << "time out ---" << std::endl;
        } else if (timer.elapsed() > timeOut) {
            return false;
        }
    }
    return true;
}

SPStatus SerialPortThread::receivePacket(uint8_t *c, uint16_t &length)
{
    SPStatus spStatus = SPStatus::RX_RECEIVING;
    static SyncState state = STATE_SYNC1;
    switch (state) {
        case STATE_SYNC1: {
            //m_recvPacket->empty();
            m_recvPacket->setSize(0);
            if (c[0] != SYNC1) {
                return SPStatus::RX_SYNCFAIL;
            }
            m_recvPacket->writeVal<uint8_t>(c[0]);
            //std::cout << "SYNC1" << std::endl;
            state = STATE_SYNC2;
            break;
        }
        case STATE_SYNC2: {
            // 为帧头2时，跳转到状态3，获取数据，否则跳回状态1
            if (c[0] != SYNC2) {
                state = STATE_SYNC1;
                return SPStatus::RX_SYNCFAIL;
            }
            m_recvPacket->writeVal<uint8_t>(c[0]);
            //std::cout << "SYNC2" << std::endl;
            state = STATE_LENGTH;
            length = 5;
            break;
        }
        case STATE_LENGTH: {
            length      = *(uint32_t *)(&c[0]) + 2;
            uint8_t v_l = c[0] + c[1] + c[2] + c[3];
            if (v_l != c[4]) {
                state = STATE_SYNC1;
                return SPStatus::RX_SYNCFAIL;
            }
            m_recvPacket->write(c, 5);
            //std::cout << "SYNC_LENGTH " << length << std::endl;
            state = STATE_ACQUIRE_DATA;
            break;
        }
        case STATE_ACQUIRE_DATA:
            m_recvPacket->write(c, length);
            auto size = m_recvPacket->getSize();
            /*for (int i = 0; i < size; i++) {
                printf("0x%x ", (uint8_t)m_recvPacket->getBuf()[i]);
            }
            printf("\n");
            std::cout << m_recvPacket->getSize() << std::endl;*/
            //if (m_recvPacket->verifyCheckSum()) {
                //std::cout << "STATE_ACQUIRE_DATA" << std::endl;
                m_recvPacket->resetRead();
                parseCommand();
                spStatus =  SPStatus::RX_COMPLETE;
            /*} else {
                std::cout << "verify fail" << std::endl;
                spStatus = SPStatus::RX_SYNCFAIL;
            }*/
            state = STATE_SYNC1;
            length = 1;
    }
    return spStatus;
}


SPStatus SerialPortThread::receiveProcess()
{
    if (m_showString.size() > 0 &&  m_showStringOutTime.elapsed() > 10) {
        emit showString(m_showString);
        //std::cout << "showString" << std::endl;
        m_showStringOutTime.start();
        m_showString.clear();
    }
    SPStatus spStatus = SPStatus::RX_IDLE;
    uint16_t length = 1;
    do {
        uint8_t* c = new uint8_t[length]();
        if (!readBuff(c, length, 200)) {
            delete[] c;
            return SPStatus::RX_IDLE;
        }
        //std::cout << std::string().copy((char*)c, length, 0) << std::endl;
        spStatus = receivePacket(c, length);
        delete[] c;

    } while (spStatus != SPStatus::RX_COMPLETE && spStatus != SPStatus::RX_SYNCFAIL);
    if (spStatus == SPStatus::RX_SYNCFAIL) {

        DataBuffer data = m_recvPacket->getBuf();
        length = m_recvPacket->getSize();
        if (data[0] != '\r') {
            //if (data[0] == 0xaa) buff.append("\n");
            QString temp;
            if (m_receiveHex) {
                for (int i = 0; i < length; i++) {
                    QByteArray hex = QByteArray(&data[i], 1).toHex();
                    temp.append(hex).append(" ");
                }
            } else {
                temp.append(QByteArray(data.get_data(), length));
            }
            m_showString.append(temp);
            if (m_showString.size() > 10) {
                emit showString(m_showString);
                //std::cout << "showString" << std::endl;
                m_showStringOutTime.start();
                m_showString.clear();
            }
        }
    }

    return spStatus;
}

SPStatus SerialPortThread::sendProcess()
{
    SPStatus sendStatus = SPStatus::TX_IDLE;
    send();
    return sendStatus;
}

void SerialPortThread::run()
{
    QTime timer;
    timer.start();
    SPStatus receiveStatus = SPStatus::RX_COMPLETE;
    while (PortStatus::open == m_portStatus) {
//        if (receiveStatus == SPStatus::RX_IDLE) {
//            if (timer.elapsed() > 1000) {
//                std::cout << "time out" << std::endl;
//                timer.start();
//                receiveStatus = receiveProcess();
//            }
//        } else {
            receiveStatus = receiveProcess();
        //}
        SPStatus sendStatus = sendProcess();

        //QThread::msleep(1);
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
    switch (m_recvPacket->getID()) {
        case UPLOAD_MAP_PACKET_ID: {
            uint8_t count = m_recvPacket->readVal<uint8_t>();
            for (int i = 0; i < count; i++) {
                uint16_t id = m_recvPacket->readVal<uint16_t>();
                uint8_t type = m_recvPacket->readVal<uint8_t>();
                uint16_t x = m_recvPacket->readVal<uint16_t>();
                uint16_t y = m_recvPacket->readVal<uint16_t>();
                uint8_t theta = m_recvPacket->readVal<uint8_t>();
                emit drawPoseData(x, y, theta, type);
                std::cout << id << " " << (int) type << " " << x << " " << y << " "
                          << (int) theta << std::endl;
            }
            break;
        }
        case UPLOAD_CURRENT_POSE_ID: {
            PoseData currentPose;
            currentPose.x = m_recvPacket->readVal<uint16_t>();
            currentPose.y = m_recvPacket->readVal<uint16_t>();
            currentPose.theta = m_recvPacket->readVal<uint8_t>();
            if (!equalsP(lastPose, currentPose) || first) {
                if (!first) {
                    //emit drawPoseData(lastPose.x, lastPose.y, lastPose.theta, 0);
                    emit drawMovePath(lastPose.x, lastPose.y, currentPose.x, currentPose.y);
                } else {
                    first = false;
                }
                lastPose.x = currentPose.x;
                lastPose.y = currentPose.y;
                lastPose.theta = currentPose.theta;
            }
           // emit drawPoseData(currentPose.x, currentPose.y, currentPose.theta, 3);
            emit updateCurPose(currentPose.x, currentPose.y, currentPose.theta);
            break;
        }
        case UPLOAD_NAV_PATH_ID : {
            std::cout << "get path ***********" << std::endl;
            PoseData prePose;
            uint8_t count = m_recvPacket->readVal<uint8_t>();
            std::vector<QPair<int, int>> navPath;
            navPath.emplace_back(QPair<int, int>(lastPose.x, lastPose.y));
            for (int i = 0; i < count; i++) {

                PoseData currentPose;
                currentPose.x = m_recvPacket->readVal<uint16_t>();
                currentPose.y = m_recvPacket->readVal<uint16_t>();
                navPath.emplace_back(QPair<int, int>(currentPose.x, currentPose.y));
//                emit drawMovePath(prePose.x, prePose.y, currentPose.x, currentPose.y, 1);
//                prePose.x = currentPose.x;
//                prePose.y = currentPose.y;
            }
            emit drawNavPath(navPath);
            break;
        }
        case UPLOAD_BOUND_ID: {
            int x1 = m_recvPacket->readVal<uint16_t>();
            int y1 = m_recvPacket->readVal<uint16_t>();
            int x2 = m_recvPacket->readVal<uint16_t>();
            int y2 = m_recvPacket->readVal<uint16_t>();
            emit drawBound(x1, y1, x2, y2, 0);
            x1 = m_recvPacket->readVal<uint16_t>();
            y1 = m_recvPacket->readVal<uint16_t>();
            x2 = m_recvPacket->readVal<uint16_t>();
            y2 = m_recvPacket->readVal<uint16_t>();
            emit drawBound(x1, y1, x2, y2, 1);
            break;
        }
        case UPLOAD_MOVE_ID: {

            break;
        }
        case UPLOAD_SENSOR_ID : {
            //std::cout << ">>>> UPLOAD_SENSOR_ID" << std::endl;
            SensorData data;
            m_recvPacket->toVal(data.encoder_t);
            m_recvPacket->toVal(data.encoder_l);
            m_recvPacket->toVal(data.encoder_r);
            m_recvPacket->toVal(data.imu_t);
            m_recvPacket->toVal(data.a_x);
            m_recvPacket->toVal(data.a_y);
            m_recvPacket->toVal(data.a_z);
            m_recvPacket->toVal(data.w_x);
            m_recvPacket->toVal(data.w_y);
            m_recvPacket->toVal(data.w_z);
            m_recvPacket->toVal(data.Roll);
            m_recvPacket->toVal(data.Pitch);
            m_recvPacket->toVal(data.Yaw);
            m_recvPacket->toVal(data.right_wall);
            m_recvPacket->toVal(data.cliff_and_bump);
            m_recvPacket->toVal(data.led_status);
            m_recvPacket->toVal(data.charge_state);
            m_recvPacket->toVal(data.current_fan);
            m_recvPacket->toVal(data.off_ground);
            m_recvPacket->toVal(data.left_piles);
            m_recvPacket->toVal(data.right_piles);
            m_recvPacket->toVal(data.current_left_wheel_avg);
            m_recvPacket->toVal(data.current_right_wheel_avg);
            m_recvPacket->toVal(data.current_roll);
            m_recvPacket->toVal(data.current_left_side);
            m_recvPacket->toVal(data.current_right_side);
            m_recvPacket->toVal(data.current_left_wheel);
            m_recvPacket->toVal(data.current_right_wheel);
            m_recvPacket->toVal(data.ir_state);
            m_recvPacket->toVal(data.current_lidar);
            m_recvPacket->toVal(data.left_wall);
            m_recvPacket->toVal(data.carpet);
            m_recvPacket->toVal(data.lidar_collide);
            m_recvPacket->toVal(data.ultrasoinc_value);
            m_recvPacket->toVal(data.front_left_piles);
            m_recvPacket->toVal(data.front_right_piles);
            m_recvPacket->toVal(data.left_front_collide);
            m_recvPacket->toVal(data.mid_front_collide);
            m_recvPacket->toVal(data.right_front_collide);
            emit sensorUpdate(data);
            break;
        }
        case UPLOAD_LOG_ID: {
            uint8_t type = m_recvPacket->readVal<uint8_t>();
            uint8_t len  = m_recvPacket->readVal<uint8_t>();
            char data[len];
            m_recvPacket->read((void *)data, len);
            emit showString(QString::fromUtf8(data, len));
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