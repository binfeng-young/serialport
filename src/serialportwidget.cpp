/**
 * Created by bfyoung on 2018/3/7.
 */

#include "serialportwidget.h"
#include "ui_serialportwidget.h"
#include "serialportthread.h"
#include <QSerialPortInfo>
#include <iostream>
#include <QGraphicsRectItem>
#include <queue>
#include <cmath>
#include <QTime>

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
        serialPortThread->reSetMap();
        drawGridMap();
    });
    connect(ui->cleanBuff, &QPushButton::clicked, [&](){
        ui->receivePlainTextEdit->clear();
    });
    connect(ui->receivePlainTextEdit, &QPlainTextEdit::textChanged, [&]() {
//        QTextCursor cursor = ui->receivePlainTextEdit->textCursor();
//        cursor.movePosition(QTextCursor::End);
        ui->receivePlainTextEdit->moveCursor(QTextCursor::End);
    });
    connect(ui->receiveHexCheckBox, SIGNAL(stateChanged(int)), serialPortThread, SLOT(onReceiveHex(int)));

    connect(ui->portsList, SIGNAL(currentIndexChanged(const QString &))
        , serialPortThread, SLOT(deviceChanged(const QString &)));
    connect(serialPortThread, SIGNAL(opened(bool)), this, SLOT(onOpened(bool)));
    connect(serialPortThread, SIGNAL(showString(const QString&)), this, SLOT(onShowString(const QString&)));
    connect(serialPortThread, SIGNAL(drawPoseData(int, int, int, int)), this, SLOT(onDrawPoseData(int, int, int, int)));
    connect(serialPortThread, SIGNAL(drawPath(int, int, int, int, int)), this, SLOT(onDrawPath(int, int, int, int, int)));
    connect(ui->sendButton, SIGNAL(clicked()), serialPortThread, SLOT(onSend()));
    getSerialPorts();
    ui->sendButton->setDisabled(true);
    ui->receivePlainTextEdit->setFont(QFont(tr("Consolas"), 11));
    //ui->scrollArea->setBackgroundRole(QPalette::Light);
    //ui->scrollArea->setWindowIconText("asdfeqwegdsgfasdgad\n\nasdfawe");
    //ui->label->setText("fasdgqwegadsgadgadflkajd;\n\n\nadsfasdfad\nb\nafadfadsf\n");
    //ui->label->text().append("fqwjerowejrowiejrowiejfaowijfalsdkfj");
    //ui->label->setTextInteractionFlags(Qt::TextSelectableByMouse); // set default
    //ui->mapView->setFrameShape (QFrame::Box);
    m_sceneSize = QSize(560, 560);
    m_cellSize = m_sceneSize / 80;
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0, 0, m_sceneSize.width(), m_sceneSize.height());
    ui->mapView->setScene(scene);
    drawGridMap();
/*    static int direction[4][2] = {0, 1, 1, 0, -1, 0, 0, -1};
    auto isOutMap = [](int x, int y, unsigned short int size_x, unsigned short int size_y)->bool {
        return x < 0 || y < 0 || x >= size_x || y >= size_y;
    };
    auto nhood4 = [&](unsigned short int idx, unsigned short int size_x, unsigned short int size_y)
    {
        //get 4-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned short int> out;
        out.reserve(8);
        int my = idx / size_x;
        int mx = idx % size_x;
        for (auto &i : direction) {
            int y = my + i[0];
            int x = mx + i[1];
            if (!isOutMap(x, y, size_x, size_y)) {
                out.push_back(y * size_x + x);
            }
        }
        return out;
    };
    auto n4 = nhood4(79 * 80 + 79, 80, 80);
    for (auto n : n4) {
        std::cout << n % 80 << "," << n/80 << std::endl;
    }*/
/*    QTime timer;
    timer.start();
    uint8_t *c = new uint8_t[6400]();
    std::cout << timer.elapsed() << std::endl;*/
}

SerialPortWidget::~SerialPortWidget()
{
    delete ui;
}

void SerialPortWidget::getSerialPorts()
{
    serialPortThread->close();
    ui->portsList->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

    // sort the list by port number (nice idea from PT_Dreamer :))
    qSort(ports.begin(), ports.end(), [](const QSerialPortInfo &s1, const QSerialPortInfo &s2) {
        return s1.portName() < s2.portName();
    });
    for (const auto &port: ports) {
        if (ui->portsList->findText(port.portName()) == -1){
            ui->portsList->addItem(port.portName());
        }
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
    ui->receivePlainTextEdit->textCursor().insertText(string);
    //std::cout << string.toStdString();
}
enum PoseType {
    FREE_SPACE,
    LETHAL_OBSTACLE,
    CURRENT,
};
void SerialPortWidget::onDrawPoseData(int x, int y, int theta, int type)
{
//    QGraphicsItem *item = ui->mapView->scene()->itemAt(
//        (79 - y) * m_cellSize.width() + 1, (79 - x) * m_cellSize.height() + 1, QTransform());
    QGraphicsRectItem *itemRect = nullptr; //dynamic_cast<QGraphicsRectItem *> (item);
    auto items = ui->mapView->scene()->items(
        QRectF((79 - y) * m_cellSize.width() - 2, (79 - x) * m_cellSize.height() - 2, m_cellSize.width() - 2,
               m_cellSize.height() - 2));
//    std::cout <<"pose " <<(79 - y) * m_cellSize.width() + 2 << " " << (79 - x) * m_cellSize.height() + 2 << " " << items.size() <<  std::endl;
    for (auto item : items) {
        if (typeid(QGraphicsRectItem) == typeid(*item)) {
            itemRect = dynamic_cast<QGraphicsRectItem *> (item);
            break;
        }
    }
    if (nullptr != itemRect) {
        QColor color(255, 255, 255);
        switch (type) {
            case PoseType::FREE_SPACE:
                //color.setRgb(0xFF6A6A);
                color.setRgb(0xFF0000);
                break;
            case PoseType::LETHAL_OBSTACLE:
                color.setRgb(0, 0, 0);
                break;
            case PoseType::CURRENT:
                color.setRgb(0, 128, 255);
                break;
//            case 3:
//                color.setRgb(0x8E8E8E);
                break;
            default:
                color.setRgb(255, 255, 255);
                break;
        }
        itemRect->setBrush(QBrush(color));
        itemRect->setPen(QPen(color));
    }
    //ui->mapView->repaint();
//    std::cout << "pose done " << std::endl;
}

void SerialPortWidget::onDrawPath(int x1, int y1, int x2, int y2, int type)
{
    QColor color(255, 255, 255);
    if (type) {
        color.setRgb(0x0000EE);
    }
    QGraphicsScene *scene = ui->mapView->scene();
    //std::cout << " lene :" << x1 << " "<< y1 << " " << x2 << " " << y2 << std::endl;
    scene->addLine((79 - y1) * m_cellSize.width() + 4, (79 - x1) * m_cellSize.height() + 4, (79 - y2) * m_cellSize.width() +4,
                   (79 - x2) * m_cellSize.height() + 4, QPen(color));
   // std::cout << "line done " << std::endl;
}

void SerialPortWidget::drawGridMap()
{
    QGraphicsScene *scene = ui->mapView->scene();
    for (int i = 0; i < 80; i++) {
        for (int j = 0; j < 80; j++) {
            scene->addRect(
                QRectF(i * m_cellSize.width(), j * m_cellSize.height(), m_cellSize.width(), m_cellSize.height()),
                QPen(QColor(255, 255, 255)),
                //QPen(QColor(204, 240, 200)),
                QBrush(QColor(255, 255, 255)));
        }
    }
}