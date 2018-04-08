//
// Created by binfeng.yang on 2018/4/7.
//

#ifndef SERIALPORT_SERIALPORTTHREAD_H
#define SERIALPORT_SERIALPORTTHREAD_H

#include <QThread>

class QSerialPort;

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

public slots:
    void onOpen();
    void deviceChanged(const QString & deviceName);
    void onReceiveHex(int checkState);

private:
    QString readBuff(int len);

private:
    QSerialPort *m_serialDevice;
    PortStatus m_portStatus;
    QString m_deviceName;
    bool m_receiveHex;

protected:
    void run() override;
};


#endif //SERIALPORT_SERIALPORTTHREAD_H
