/**
 * Created by bfyoung on 2018/3/7.
 */

#include "serialportwidget.h"
#include "ui_serialportwidget.h"
#include "serialportthread.h"
#include <QSerialPortInfo>
#include <iostream>
#include <QGraphicsRectItem>

SerialPortWidget::SerialPortWidget(QWidget *parent)
    :
    QWidget(parent), ui(new Ui::SerialPortWidget)
{
    ui->setupUi(this);
    ui->refreshPorts->setIcon(QIcon(":images/view-refresh.svg"));
    serialPortThread = new SerialPortThread();
    connect(ui->openButton, SIGNAL(clicked()), serialPortThread, SLOT(onOpen()));
    connect(ui->refreshPorts, SIGNAL(clicked()), this, SLOT(getSerialPorts()));
    connect(ui->cleanMapButton, &QPushButton::clicked, [&](){
        ui->mapView->scene()->clear();
        drawGridMap();
    });
    connect(ui->receiveTextEdit, &QTextEdit::textChanged, [&]() {
        QTextCursor cursor = ui->receiveTextEdit->textCursor();
        cursor.movePosition(QTextCursor::End);
        ui->receiveTextEdit->setTextCursor(cursor);
    });
    connect(ui->receiveHexCheckBox, SIGNAL(stateChanged(int)), serialPortThread, SLOT(onReceiveHex(int)));

    connect(ui->portsList, SIGNAL(currentIndexChanged(const QString &))
        , serialPortThread, SLOT(deviceChanged(const QString &)));
    connect(serialPortThread, SIGNAL(opened(bool)), this, SLOT(onOpened(bool)));
    connect(serialPortThread, SIGNAL(showString(const QString&)), this, SLOT(onShowString(const QString&)));
    connect(serialPortThread, SIGNAL(drawPoseData(int, int, int, int)), this, SLOT(onDrawPoseData(int, int, int, int)));
    connect(serialPortThread, SIGNAL(drawPath(int, int, int, int)), this, SLOT(onDrawPath(int, int, int, int)));
    //connect(ui->sendButton, SIGNAL(clicked()), this, SLOT(onSend()));
    getSerialPorts();
    ui->sendButton->setDisabled(true);
    ui->receiveTextEdit->setFont(QFont(tr("Consolas"), 11));
    //ui->mapView->setFrameShape (QFrame::Box);
    m_sceneSize = QSize(560, 560);
    m_gridSize = m_sceneSize / 80;
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0, 0, m_sceneSize.width(), m_sceneSize.height());
    ui->mapView->setScene(scene);
    drawGridMap();
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
    } else {
        ui->openButton->setText(tr("open"));
        ui->sendButton->setDisabled(true);
    }
//
}

void SerialPortWidget::onShowString(const QString &string)
{
    ui->receiveTextEdit->insertPlainText(string);
}
enum PoseType {
    FREE_SPACE,
    LETHAL_OBSTACLE,
    CURRENT,
};
void SerialPortWidget::onDrawPoseData(int x, int y, int theta, int type)
{
    QGraphicsRectItem *item = reinterpret_cast<QGraphicsRectItem *>(
        ui->mapView->scene()->itemAt((79 - y) * m_gridSize.width(), (79 - x) * m_gridSize.height(), QTransform()));
    QColor *color = nullptr;
    switch (type) {
        case PoseType::FREE_SPACE:
            color = new QColor(255, 0, 0);
            break;
        case PoseType::LETHAL_OBSTACLE:
            color = new QColor(0, 0, 0);
            break;
        case PoseType::CURRENT:
            color = new QColor(0, 128, 255);
            break;
//        case 3:
//            color = new QColor(128, 255, 0);
//            break;
        default:
            color = new QColor(255, 255, 255);
            break;
    }
    item->setBrush(QBrush(*color));
    //ui->mapView->repaint();
    delete color;
}

void SerialPortWidget::onDrawPath(int x1, int y1, int x2, int y2)
{
    QGraphicsScene *scene = ui->mapView->scene();
    scene->addLine((79 -y1) * m_gridSize.width(), (79 - x1) * m_gridSize.height(), (79 - y2) * m_gridSize.width(),
                   (79 - x2) * m_gridSize.height(), QPen(QColor(128, 255, 0)));
}

void SerialPortWidget::drawGridMap()
{
    QGraphicsScene *scene = ui->mapView->scene();
    for (int i = 1; i <= 80; i++) {
        for (int j = 1; j <= 80; j++) {
            scene->addRect(
                QRectF(i * m_gridSize.width(), j * m_gridSize.height(), m_gridSize.width(), m_gridSize.height()),
                QPen(QColor(204, 240, 200)),
                QBrush(QColor(255, 255, 255)));
        }
    }
}