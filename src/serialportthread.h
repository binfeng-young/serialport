//
// Created by binfeng.yang on 2018/4/7.
//

#ifndef SERIALPORT_SERIALPORTTHREAD_H
#define SERIALPORT_SERIALPORTTHREAD_H

#include <QThread>
#include <mutex>
#include <QSerialPort>
#include <QtCore/QTime>
#include <queue>

class Packet;
enum class SPStatus {
    TX_IDLE,
    TX_WAITING,
    TX_TIMEOUT,
    TX_ACKED,
    TX_BUFOVERRUN,
    TX_BUSY,
    RX_IDLE,
    RX_SYNCFAIL,
    RX_RECEIVING,
    RX_COMPLETE,
};
class SerialPortThread : public QThread {
Q_OBJECT
    enum class PortStatus {
        open, closed, error
    };
public:
    SerialPortThread();

    ~SerialPortThread();

    void open(const QString& deviceName);

    void close();

    void reSetMap();
signals:
    void opened(bool opened);
    void showString(const QString& string);
    void drawPoseData(int x, int y, int theta, int type);
    void drawPath(int x1, int y1, int x2, int y2, int type);

public slots:
    void onOpen();
    void deviceChanged(const QString & deviceName);
    void onReceiveHex(int checkState);
    void handleSerialError(QSerialPort::SerialPortError error);
    void onSend(const QByteArray &text);
//    void onRead();
    void readFromSerial();

private:
    SPStatus receiveProcess();
    SPStatus receivePacket(uint8_t *c, uint16_t &length);
    SPStatus sendProcess();

    bool readChar(char &c);
    bool readBuff(void *data, int len, int timeOut);
    void parseCommand();
    bool writChar(char &c);
    bool send();
    void sendBuff(const QByteArray&);

private:
    QSerialPort *m_serialDevice;
    Packet* m_recvPacket;
    PortStatus m_portStatus;
    QString m_deviceName;
    QString m_showString;
    QTime m_showStringOutTime;
    bool m_receiveHex;
    //QMutex m_serialMutex;
    std::mutex sendbufmutex;
    bool m_sendPending;
    char * m_sendBuf;
    uint16_t m_sendLen;
    QByteArray m_readArray;
    QByteArray m_writeArray;
    int m_queueOffSet;
protected:
    void run() override;
};


#endif //SERIALPORT_SERIALPORTTHREAD_H
