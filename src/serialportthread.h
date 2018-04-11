//
// Created by binfeng.yang on 2018/4/7.
//

#ifndef SERIALPORT_SERIALPORTTHREAD_H
#define SERIALPORT_SERIALPORTTHREAD_H

#include <QThread>
#include <mutex>
#include <QSerialPort>

class Packet;

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

signals:
    void opened(bool opened);
    void showString(const QString& string);
    void drawPoseData(int x, int y, int theta, int type);
    void drawPath(int x1, int y1, int x2, int y2);

public slots:
    void onOpen();
    void deviceChanged(const QString & deviceName);
    void onReceiveHex(int checkState);
    void handleSerialError(QSerialPort::SerialPortError error);

private:
    bool readChar(char *c);
    bool readBuff(void *data, int len);
    void parseCommand();

private:
    QSerialPort *m_serialDevice;
    Packet* m_recv_packet;
    PortStatus m_portStatus;
    QString m_deviceName;
    bool m_receiveHex;

protected:
    void run() override;
};


#endif //SERIALPORT_SERIALPORTTHREAD_H
