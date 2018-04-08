/**
 * Created by bfyoung on 2018/3/7.
 */

#include "serialportwidget.h"
#include "ui_serialportwidget.h"
#include "serialportthread.h"
#include <QSerialPortInfo>

SerialPortWidget::SerialPortWidget(QWidget *parent)
    :
    QWidget(parent), ui(new Ui::SerialPortWidget)
{
    ui->setupUi(this);
    ui->refreshPorts->setIcon(QIcon(":images/view-refresh.svg"));
    serialPortThread = new SerialPortThread();
    connect(ui->openButton, SIGNAL(clicked()), serialPortThread, SLOT(onOpen()));
    connect(ui->refreshPorts, SIGNAL(clicked()), this, SLOT(getSerialPorts()));
    connect(ui->receiveTextEdit, &QTextEdit::textChanged, [&](){
        QTextCursor cursor=ui->receiveTextEdit->textCursor();
        cursor.movePosition(QTextCursor::End);
        ui->receiveTextEdit->setTextCursor(cursor);
    });

    connect(ui->portsList, SIGNAL(currentIndexChanged(const QString &))
        , serialPortThread, SLOT(deviceChanged(const QString &)));
    connect(serialPortThread, SIGNAL(opened(bool)), this, SLOT(onOpened(bool)));
    connect(serialPortThread, SIGNAL(showString(const QString&)), this, SLOT(onShowString(const QString&)));
    //connect(ui->sendButton, SIGNAL(clicked()), this, SLOT(onSend()));
    getSerialPorts();
    ui->sendButton->setDisabled(true);
    ui->receiveTextEdit->setFont(QFont(tr("Consolas"), 11));
}

SerialPortWidget::~SerialPortWidget()
{
    delete ui;
}

void SerialPortWidget::getSerialPorts()
{
    serialPortThread->close();
    // Populate the telemetry combo box:
    ui->portsList->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

    // sort the list by port number (nice idea from PT_Dreamer :))
    qSort(ports.begin(), ports.end(), [](const QSerialPortInfo &s1, const QSerialPortInfo &s2) {
        return s1.portName() < s2.portName();
    });
    foreach(QSerialPortInfo port, ports) {
        ui->portsList->addItem(port.portName());
    }

}

void SerialPortWidget::onOpened(bool opened)
{
    if (opened) {
        ui->openButton->setText(tr("close"));
        ui->sendButton->setDisabled(false);
    } else  {
        ui->openButton->setText(tr("open"));
        ui->sendButton->setDisabled(true);
    }
//
}

void SerialPortWidget::onShowString(const QString& string)
{
    ui->receiveTextEdit->insertPlainText(string);
}