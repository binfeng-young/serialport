/**
 * Created by bfyoung on 2018/3/7.
 */

#include "serialportwidget.h"
#include "ui_serialportwidget.h"
#include "serialportthread.h"
#include "map_thread.h"
#include <QSerialPortInfo>
#include <iostream>
#include <QGraphicsRectItem>
#include <queue>
#include <cmath>
#include <QTime>
#include <QMouseEvent>
#include <QDebug>
#include <QScrollBar>
#include <memory>
#include <future>
using namespace bv::mapping;

SerialPortWidget::SerialPortWidget(QWidget *parent)
    :
    QWidget(parent), ui(new Ui::SerialPortWidget)
{
    ui->setupUi(this);
    qRegisterMetaType<std::vector<QPair<int, int>>>("std::vector<QPair<int, int>>");
    scale_ = 1;
    mapSize_ = QSize(80, 80);
    m_sceneSize = QSize(1200, 1200);
    m_cellSize = QSize(m_sceneSize.width() / mapSize_.width(), m_sceneSize.height() / mapSize_.height());
    map_thread_ = std::make_shared<MapThread>(mapSize_.width(), mapSize_.height());
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0, 0, m_sceneSize.width(), m_sceneSize.height());
    ui->mapView->setScene(scene);

    ui->mapView->setStyleSheet("border:none; background:lightgray;");
    ui->mapView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->mapView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    serialPortThread = new SerialPortThread();
    connect(ui->openButton, SIGNAL(clicked()), serialPortThread, SLOT(onOpen()));
    connect(ui->clean_map_button, &QPushButton::clicked, [&](){
        ui->mapView->scene()->clear();
        serialPortThread->reSetMap();
        drawGridMap();
    });
    //setMouseTracking(true);

    ui->mapView->setMouseTracking(true);

    connect(map_thread_.get(), SIGNAL(drawMap(int, int, int)), this, SLOT(onDrawMap(int, int, int)));
    connect(map_thread_.get(), SIGNAL(drawPoseData(int, int, int, int)), this, SLOT(onDrawPoseData(int, int, int, int)));
    connect(map_thread_.get(), SIGNAL(drawMovePath(int, int, int, int)), this, SLOT(onDrawMovePath(int, int, int, int)));
    //connect(serialPortThread, SIGNAL(drawNavPath(std::vector<QPair<int, int>>)), this, SLOT(onDrawNavPath(std::vector<QPair<int, int>>)));
    connect(map_thread_.get(), SIGNAL(updateCurPose(int, int, int)), this, SLOT(onUpdateCurPose(int, int, int)));
    //connect(serialPortThread, SIGNAL(drawBound(int, int, int, int, int)), this, SLOT(onDrawBound(int, int, int, int, int)));
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

enum MapInfoType {
    FREE_SPACE,
    LETHAL_OBSTACLE,
    INSCRIBED_INFLATED_OBSTACLE,
};

void SerialPortWidget::onDrawMap(int x, int y, int val) {
    QGraphicsRectItem *itemRect = gridMap_[x][y];
    auto ch = (QGraphicsTextItem*)itemRect->childItems()[0];
    ch->setPlainText(QString("%1").arg(val));
}

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
    QGraphicsRectItem *itemRect = gridMap_[x][y];
    itemRect->setBrush(QBrush(color));
    itemRect->setPen(QPen(color));
    //ui->mapView->repaint();
//    std::cout << "pose done " << std::endl;
}

void SerialPortWidget::onUpdateCurPose(int x, int y, int theta) {

    QGraphicsRectItem *itemRect = gridMap_[x][y];
    //itemRect->pos();
    auto p1 = itemRect->mapFromScene(itemRect->boundingRect().center());
    //int xx =
    //int yy = (mapSize_.height() -1 - x) * m_cellSize.height() + 3;
    int width = m_cellSize.width()*3;
    int height = m_cellSize.height()*3;

    QGraphicsScene *scene = ui->mapView->scene();
    delete curPose_;
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
        delete curRegion_;
        curRegion_ = scene->addRect(QRectF(p1.x(), p1.y(), width, height), QPen(QColor(0xFF00FF))
            , QBrush(QColor(255, 255, 255, 0)));

    } else {
        delete bound_;
        bound_ = scene->addRect(QRectF(p1.x(), p1.y(), width, height), QPen(QColor(0x8B6508))
            , QBrush(QColor(255, 255, 255, 0)));
    }
}

void SerialPortWidget::onDrawNavPath(std::vector<QPair<int, int>> navPath)
{
    if (navPath.empty()) {
        return;
    }
    std::cout << "on Draw nav path " << navPath.size() << std::endl;
/*    for (auto &path : navPath_) {
        delete path;
        path = nullptr;
    }*/
    std::cout << "nav path size: " << navPath.size() << std::endl;
    navPath_.clear();
    QColor color(0x0000EE);
    QGraphicsScene *scene = ui->mapView->scene();
    int x1 = navPath[0].first;
    int y1 = navPath[0].second;
    for (int i = 1; i < navPath.size(); i++) {
        int x2 = navPath[i].first;
        int y2 = navPath[i].second;
        navPath_.push_back(scene->addLine((mapSize_.width() -1 - y1 + 0.5) * m_cellSize.width(), (mapSize_.height() -1 - x1+ 0.5) * m_cellSize.height(), (mapSize_.width() -1 - y2+ 0.5) * m_cellSize.width(),
                                          (mapSize_.height() -1 - x2+ 0.5) * m_cellSize.height(), QPen(QBrush(color), 2)));
        x1 = x2;
        y1 = y2;
    }

}

void SerialPortWidget::onDrawMovePath(int x1, int y1, int x2, int y2)
{
    QColor color(0x00FF00);
    QGraphicsScene *scene = ui->mapView->scene();
    //std::cout << " lene :" << x1 << " "<< y1 << " " << x2 << " " << y2 << std::endl;
    scene->addLine((x1 + 0.5) * m_cellSize.width(), (y1+ 0.5) * m_cellSize.height(), (x2+ 0.5) * m_cellSize.width(),
                   (y2+ 0.5) * m_cellSize.height(), QPen(color));
   // std::cout << "line done " << std::endl;
}

void SerialPortWidget::drawGridMap()
{
    curPose_ = nullptr;
    curRegion_ = nullptr;
    bound_ = nullptr;
    gridMap_.clear();
    QGraphicsScene *scene = ui->mapView->scene();
    QFont font;
    font.setPointSize(6);
    for (int i = 0; i < mapSize_.width(); i++) {
        std::vector<QGraphicsRectItem*> gridMapW;
        gridMapW.reserve(static_cast<unsigned long>(mapSize_.height()));
        for (int j = 0; j < mapSize_.height(); j++) {
            QPoint p(i * m_cellSize.width(), j * m_cellSize.height());
            QSizeF size(m_cellSize.width(), m_cellSize.height());
            auto item = scene->addRect(
                QRectF(p, size),
                //QPen(QColor(255, 255, 255)),
                QPen(QColor(255, 255, 255)),
                QBrush(QColor(255, 255, 255)));
            gridMapW.push_back(item);
            QGraphicsTextItem* text = new QGraphicsTextItem(item);
            text->setPlainText(QString("%1").arg(0));
            QRectF rect = text->boundingRect();
            text->setDefaultTextColor(QColor(0xDC143C));
            p.setX(p.x() + m_cellSize.width()/2 - rect.width()/2);
            text->setPos(p);
            text->setFont(font);
        }
        gridMap_.push_back(gridMapW);
    }

    map_thread_->start();
}
/*
void SerialPortWidget::wheelEvent(QWheelEvent *event)
{

}

void SerialPortWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPointF = event->pos();
    qDebug() << m_lastPointF.x() << m_lastPointF.y();
}
void SerialPortWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_lastPointF = QPointF(0, 0);
}

void SerialPortWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_lastPointF == QPointF(0, 0)) {
        return;
    }
    QPointF disPointF = event->pos() - m_lastPointF;
    qDebug() << disPointF.x() << disPointF.y();
    m_lastPointF = event->pos();
    ui->mapView->scene()->setSceneRect(ui->mapView->scene()->sceneRect().x()+disPointF.x(),ui->mapView->scene()->sceneRect().y()+disPointF.y(),
        ui->mapView->scene()->sceneRect().width(),ui->mapView->scene()->sceneRect().height());
    ui->mapView->scene()->update();
}

void SerialPortWidget::mouseDoubleClickEvent(QMouseEvent *event)
{

}
*/
