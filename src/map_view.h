//
// Created by binfeng.yang on 2019/5/30.
//

#ifndef SERIALPORT_MAP_VIEW_H
#define SERIALPORT_MAP_VIEW_H

#include <QGraphicsView>

class MapView : public QGraphicsView {
Q_OBJECT
public:
    MapView(QWidget *parent = nullptr) : QGraphicsView(parent), m_lastPointF(QPointF(0,0)) {}
    ~MapView() {}

    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    QPointF m_lastPointF;
};


#endif //SERIALPORT_MAP_VIEW_H
