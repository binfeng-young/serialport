//
// Created by binfeng.yang on 2019/5/30.
//

#include "map_view.h"

#include <QMouseEvent>
#include <QScrollBar>
void MapView::wheelEvent(QWheelEvent *event)
{
        const QPointF p0scene = mapToScene(event->pos());
    //qreal factor = std::pow(1.01, event->delta());
    int wheelDeltaValue = event->delta();
    if (!wheelDeltaValue) {
        return;
    }
    // 向上滚动，放大;
    if (wheelDeltaValue < 0) {
        scale(1.1, 1.1);
    }
        // 向下滚动，缩小;
    else {
        scale(1.0 / 1.1, 1.0 / 1.1);
    }
    //scale(factor, factor);
    const QPointF p1mouse = mapFromScene(p0scene);
    const QPointF move = p1mouse - event->pos(); // The move
    horizontalScrollBar()->setValue(move.x() + horizontalScrollBar()->value());
    verticalScrollBar()->setValue(move.y() + verticalScrollBar()->value());

/*    // 获取当前鼠标相对于view的位置;
    QPointF cursorPoint = event->pos();
    // 获取当前鼠标相对于scene的位置;
    QPointF scenePos = mapToScene(QPoint(cursorPoint.x(), cursorPoint.y()));
    centerOn(scenePos);

    // 获取view的宽高;
    qreal viewWidth = viewport()->width();
    qreal viewHeight = viewport()->height();

    // 获取当前鼠标位置相当于view大小的横纵比例;
    qreal hScale = cursorPoint.x() / viewWidth;
    qreal vScale = cursorPoint.y() / viewHeight;

    // 当前放缩倍数;
    qreal scaleFactor = matrix().m11();
    int wheelDeltaValue = event->delta();
    if (!wheelDeltaValue) {
        return;
    }
    // 向上滚动，放大;
    if (wheelDeltaValue < 0) {
        scale(1.1, 1.1);
    }
        // 向下滚动，缩小;
    else {
        scale(1.0 / 1.1, 1.0 / 1.1);
    }

    // 将scene坐标转换为放大缩小后的坐标;
    QPointF viewPoint = matrix().map(scenePos);
    // 通过滚动条控制view放大缩小后的展示scene的位置;
    horizontalScrollBar()->setValue(int(viewPoint.x() - viewWidth * hScale));
    verticalScrollBar()->setValue(int(viewPoint.y() - viewHeight * vScale));*/
}

void MapView::mousePressEvent(QMouseEvent *event)
{
    m_lastPointF = event->pos();
}

void MapView::mouseMoveEvent(QMouseEvent *event)
{
    if (m_lastPointF == QPointF(0, 0)) {
        return;
    }
    QPointF disPointF = m_lastPointF - event->pos();
    m_lastPointF = event->pos();
    scene()->setSceneRect(scene()->sceneRect().x()+disPointF.x(),scene()->sceneRect().y()+disPointF.y(),
                                       scene()->sceneRect().width(),scene()->sceneRect().height());
    scene()->update();
}

void MapView::mouseDoubleClickEvent(QMouseEvent *event)
{
    QGraphicsView::mouseDoubleClickEvent(event);
}

void MapView::mouseReleaseEvent(QMouseEvent *event)
{
    m_lastPointF = QPointF(0, 0);
}