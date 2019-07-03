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
    qRegisterMetaType<std::vector<QPair<int, int>>>("std::vector<QPair<int, int>>");
    mapSize_ = QSize(144, 144);
    m_sceneSize = QSize(864, 864);
    m_cellSize = QSize(m_sceneSize.width() / mapSize_.width(), m_sceneSize.height() / mapSize_.height());
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0, 0, m_sceneSize.width(), m_sceneSize.height());
    ui->mapView->setScene(scene);
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
    connect(serialPortThread, SIGNAL(drawMovePath(int, int, int, int)), this, SLOT(onDrawMovePath(int, int, int, int)));
    connect(serialPortThread, SIGNAL(drawNavPath(std::vector<QPair<int, int>>)), this, SLOT(onDrawNavPath(std::vector<QPair<int, int>>)));
    connect(serialPortThread, SIGNAL(updateCurPose(int, int, int)), this, SLOT(onUpdateCurPose(int, int, int)));
    connect(serialPortThread, SIGNAL(drawBound(int, int, int, int, int)), this, SLOT(onDrawBound(int, int, int, int, int)));
    connect(ui->sendButton, SIGNAL(clicked()), this, SLOT(onSend()));
    getSerialPorts();
    ui->sendButton->setDisabled(true);
    ui->receivePlainTextEdit->setFont(QFont(tr("Consolas"), 11));
    //ui->scrollArea->setBackgroundRole(QPalette::Light);
    //ui->scrollArea->setWindowIconText("asdfeqwegdsgfasdgad\n\nasdfawe");
    //ui->label->setText("fasdgqwegadsgadgadflkajd;\n\n\nadsfasdfad\nb\nafadfadsf\n");
    //ui->label->text().append("fqwjerowejrowiejrowiejfaowijfalsdkfj");
    //ui->label->setTextInteractionFlags(Qt::TextSelectableByMouse); // set default
    //ui->mapView->setFrameShape (QFrame::Box);
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
enum MapInfoType {
    FREE_SPACE,
    LETHAL_OBSTACLE,
    INSCRIBED_INFLATED_OBSTACLE,
};
void SerialPortWidget::onDrawPoseData(int x, int y, int theta, int type)
{

//    QGraphicsRectItem *itemRect = nullptr; //dynamic_cast<QGraphicsRectItem *> (item);
//    auto items = ui->mapView->scene()->items(
//        QRectF((143 - y) * m_cellSize.width() - 2, (143 - x) * m_cellSize.height() - 2, m_cellSize.width() - 2,
//               m_cellSize.height() - 2));
//    for (auto item : items) {
//        if (typeid(QGraphicsRectItem) == typeid(*item)) {
//            itemRect = dynamic_cast<QGraphicsRectItem *> (item);
//            break;
//        }
//    }

    QColor color(255, 255, 255);
    switch (type) {
        case MapInfoType::FREE_SPACE:
            //color.setRgb(0xFF6A6A);
            //color.setRgb(0xFF0000);
            color.setRgb(0xA4D3EE);
            break;
        case MapInfoType::LETHAL_OBSTACLE:
            color.setRgb(0, 0, 0);
            break;
        case MapInfoType::INSCRIBED_INFLATED_OBSTACLE:
            color.setRgb(0, 128, 255);
            break;
        default:
            color.setRgb(255, 255, 255);
            break;
    }
    QGraphicsRectItem *itemRect = gridMap_[mapSize_.width() -1 - y][mapSize_.height() -1 - x];
    itemRect->setBrush(QBrush(color));
    itemRect->setPen(QPen(color));
    //ui->mapView->repaint();
//    std::cout << "pose done " << std::endl;
}

void SerialPortWidget::onUpdateCurPose(int x, int y, int theta) {

    QGraphicsRectItem *itemRect = gridMap_[mapSize_.width() - 1 - y][mapSize_.height() - 1 - x];
    //itemRect->pos();
    auto p1 = itemRect->mapFromScene(itemRect->boundingRect().center());
    //int xx =
    //int yy = (mapSize_.height() -1 - x) * m_cellSize.height() + 3;
    int width = m_cellSize.width()*3;
    int height = m_cellSize.height()*3;

    QGraphicsScene *scene = ui->mapView->scene();
    scene->removeItem(curPose_);
    curPose_ = scene->addEllipse(QRectF(p1.x() - width / 2, p1.y() - height / 2, width, height), QPen(QBrush(QColor(0, 128, 255)),2)
        , QBrush(QColor(255, 255, 255, 20)));

    //curPose_->setOpacity(0.5);
    //curPose_->brush().color().rgba()
}


void SerialPortWidget::onDrawBound(int maxx, int maxy, int minx, int miny, int type)
{
    QGraphicsScene *scene = ui->mapView->scene();
    QGraphicsRectItem *itemRect = gridMap_[mapSize_.width() - 1 - maxy][mapSize_.height() - 1 - maxx];
    //itemRect->pos();
    auto p1 = itemRect->mapFromScene(itemRect->boundingRect().center());
    int width = m_cellSize.width() * abs(maxy - miny);
    int height = m_cellSize.height() * abs(maxx - minx);
    if (!type) {
        scene->removeItem(curRegion_);
        curRegion_ = scene->addRect(QRectF(p1.x(), p1.y(), width, height), QPen(QColor(0xFF00FF))
            , QBrush(QColor(255, 255, 255, 0)));

    } else {
        scene->removeItem(bound_);
        bound_ = scene->addRect(QRectF(p1.x(), p1.y(), width, height), QPen(QColor(0x8B6508))
            , QBrush(QColor(255, 255, 255, 0)));
    }
}

void SerialPortWidget::onDrawNavPath(std::vector<QPair<int, int>> navPath)
{
    if (navPath.empty()) {
        return;
    }
    std::cout << "on Draw nav path" << navPath.size() << std::endl;

    QGraphicsScene *scene = ui->mapView->scene();
    for (auto &path : navPath_) {
        //delete path;
        scene->removeItem(path);
        //path = nullptr;
    }
    navPath_.clear();
    std::cout << "nav path size: " << navPath.size() << std::endl;
    QColor color(0x0000EE);
    int x1 = navPath[0].first;
    int y1 = navPath[0].second;
    for (int i = 1; i < navPath.size(); i++) {
        int x2 = navPath[i].first;
        int y2 = navPath[i].second;
        navPath_.push_back(scene->addLine((mapSize_.width() -1 - y1) * m_cellSize.width() + 3, (mapSize_.height() -1 - x1) * m_cellSize.height() + 3, (mapSize_.width() -1 - y2) * m_cellSize.width() +3,
                                          (mapSize_.height() -1 - x2) * m_cellSize.height() + 3, QPen(QBrush(color), 2)));
        x1 = x2;
        y1 = y2;
    }

}

void SerialPortWidget::onDrawMovePath(int x1, int y1, int x2, int y2)
{
    QColor color(255, 255, 255);
    QGraphicsScene *scene = ui->mapView->scene();
    //std::cout << " lene :" << x1 << " "<< y1 << " " << x2 << " " << y2 << std::endl;
    scene->addLine((mapSize_.width() -1 - y1) * m_cellSize.width() + 3, (mapSize_.height() - 1 - x1) * m_cellSize.height() + 3, (mapSize_.width() - 1 - y2) * m_cellSize.width() + 3,
                   (mapSize_.height() -1 - x2) * m_cellSize.height() + 3, QPen(color));
   // std::cout << "line done " << std::endl;
}

void SerialPortWidget::drawGridMap()
{
    curPose_ = nullptr;
    curRegion_ = nullptr;
    bound_ = nullptr;
    gridMap_.clear();
    QGraphicsScene *scene = ui->mapView->scene();
    for (int i = 0; i < mapSize_.width(); i++) {
        std::vector<QGraphicsRectItem*> gridMapW;
        for (int j = 0; j < mapSize_.height(); j++) {
            gridMapW.push_back(scene->addRect(
                QRectF(i * m_cellSize.width(), j * m_cellSize.height(), m_cellSize.width(), m_cellSize.height()),
                QPen(QColor(255, 255, 255)),
                //QPen(QColor(204, 240, 200)),
                QBrush(QColor(255, 255, 255))));
        }
        gridMap_.push_back(gridMapW);
    }
}

void SerialPortWidget::onSend()
{
    auto text = ui->sendPlainTextEdit->toPlainText();
    if (text.isEmpty()) {
        return;
    }
    auto convertHexChar = [](char ch) {
        if((ch >= '0') && (ch <= '9'))
            return ch - 0x30;
        else if((ch >= 'A') && (ch <= 'F'))
            return ch - 'A' + 10;
        else if((ch >= 'a') && (ch <= 'f'))
            return ch - 'a' + 10;
        else return -1;
    };

    QByteArray sendData;
    int len = text.length();
    for(int i = 0; i < len; i++) {
        char hBit = text[i].toLatin1();
        if (hBit == ' ') {
            continue;
        }
        i++;
        char lBit = '0';
        if (i < len) {
            lBit = text[i].toLatin1();
        }
        if (i >= len || lBit == ' ') {
            lBit = hBit;
            hBit = '0';
        }
        int hexData = convertHexChar(hBit);
        int lowHexData = convertHexChar(lBit);
        //std::cout << hexData << " " << lowHexData <<std::endl;
        sendData.append((char)(hexData * 16 + lowHexData));
    }
    //auto s  =// string.toLatin1();//qString2Hex(string);
/*        std::cout << sendData.size() << std::endl;
    for (auto c : sendData) {
        std::cout <<(int)(uint8_t)c << " ";
    }
    std::cout << std::endl;*/
    serialPortThread->onSend(sendData);
}