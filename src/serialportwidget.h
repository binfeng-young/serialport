/**
 * Created by bfyoung on 2018/3/7.
 */

#ifndef SERIALPORTWIDGET_H
#define SERIALPORTWIDGET_H

#include <QtWidgets/QWidget>
#include <QThread>

class SerialPortThread;

namespace Ui {
    class SerialPortWidget;
}
class QSerialPort;

class SerialPortWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SerialPortWidget(QWidget *parent = 0);
    ~SerialPortWidget();

public:

signals:
public slots:
    void getSerialPorts();
    void onOpened(bool opened);
    void onShowString(const QString& string);
    void onDrawPoseData(int x, int y, int theta, int type);
private:
    Ui::SerialPortWidget* ui;
    SerialPortThread* serialPortThread;
};
#endif //SERIALPORTWIDGET_H
