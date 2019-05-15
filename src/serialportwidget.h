/**
 * Created by bfyoung on 2018/3/7.
 */

#ifndef SERIALPORTWIDGET_H
#define SERIALPORTWIDGET_H

#include <QtWidgets/QWidget>
#include <QThread>
class SerialPortThread;
class MapThread;
namespace Ui {
    class SerialPortWidget;
}
class QSerialPort;
class QGraphicsRectItem;
class QGraphicsLineItem;
class QGraphicsEllipseItem;

class SerialPortWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SerialPortWidget(QWidget *parent = 0);
    ~SerialPortWidget();
    void drawGridMap();

public:
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);


signals:
public slots:
    void onDrawPoseData(int x, int y, int theta, int type);
    void onDrawMovePath(int x1, int y1, int x2, int y2);
    void onDrawNavPath(std::vector<QPair<int, int>> navPath);
    void onUpdateCurPose(int x, int y, int theta);
    void onDrawBound(int maxx, int maxy, int minx, int miny, int type);
private:
    Ui::SerialPortWidget* ui;
    SerialPortThread* serialPortThread;
    std::shared_ptr<MapThread> map_thread_;
    QSize mapSize_;
    QSize m_sceneSize;
    QSize m_cellSize;
    QGraphicsEllipseItem* curPose_;
    QGraphicsRectItem* curRegion_;
    QGraphicsRectItem* bound_;
    std::vector<std::vector<QGraphicsRectItem*>> gridMap_;
    std::vector<QGraphicsLineItem*> navPath_;
    QPointF m_lastPointF;
    qreal scale_;
};
#endif //SERIALPORTWIDGET_H
