/**
 * Created by bfyoung on 2018/3/7.
 */

#ifndef UPLOADER_SERIALPORTWIDGET_H
#define UPLOADER_SERIALPORTWIDGET_H

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
private:
    Ui::SerialPortWidget* ui;
    SerialPortThread* serialPortThread;
};
#endif //UPLOADER_SERIALPORTWIDGET_H
