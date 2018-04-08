//
// Created by binfeng.yang on 2018/4/7.
//

#include "serialportthread.h"
#include <QSerialPort>
#include <iostream>

SerialPortThread::SerialPortThread()
    : m_serialDevice(nullptr)
      , m_portStatus(PortStatus::closed)
{

}

SerialPortThread::~SerialPortThread()
{

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
        start();
        // connect(m_serialDevice, SIGNAL(readyRead()), this, SLOT(onRead()));
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
        m_serialDevice->deleteLater();
        m_serialDevice = nullptr;
        m_portStatus = PortStatus::closed;
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

void SerialPortThread::run()
{
    QString buff;
    while (PortStatus::open == m_portStatus) {
        char c;
        m_serialDevice->waitForBytesWritten(1);
        if (m_serialDevice->bytesAvailable() || m_serialDevice->waitForReadyRead()) {
            m_serialDevice->read(&c, 1);
            if (c != '\r') {
                buff.append(c);
                if (buff.length() >= 10 && c == '\n') {
                    emit showString(QString(buff));
                    buff.clear();
                }
            }
        }
        QThread::msleep(1);
    }
}

void SerialPortThread::deviceChanged(const QString &deviceName)
{
    m_deviceName = deviceName;
}