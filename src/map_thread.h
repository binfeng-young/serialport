//
// Created by binfeng.yang on 2019/5/14.
//

#ifndef SERIALPORT_MAP_THREAD_H
#define SERIALPORT_MAP_THREAD_H
#include <QThread>
#include <QtCore/QTime>
#include <cell_index.h>
#include "typedef.h"
namespace bv {
namespace mapping {
class Costmap2D;
}
}
class MapThread : public QThread {
Q_OBJECT
public:
    MapThread(int size_x, int size_y);
    ~MapThread(){}

    void bfs(const bv::mapping::CellIndex& cur_index);
    void dfs(const bv::mapping::CellIndex& cur_index, int dir);
signals:
    void drawMap(int x, int y, int val);
    void updateCurPose(int x, int y, int theta);
    void drawMovePath(int x1, int y1, int x2, int y2);
    void drawPoseData(int x, int y, int theta, int type);
protected:
    void run() override ;
private:
    std::shared_ptr<bv::mapping::Costmap2D> map_;
    qreal scale_;
    bv::mapping::CellIndex start_index_ = bv::mapping::CellIndex(0, 0);
    std::vector<int> vis_;
    std::vector<int> vis2_;
    bv::mapping::CellIndex last_pose_index_;
    std::vector<bv::mapping::Point> triangle_;
    std::vector<bv::mapping::Point> rectangle_;
    std::vector<bv::mapping::Point> circle_;
    std::vector<bv::mapping::Point> semicircle_;
    std::vector<bv::mapping::CellIndex> ins_;
};

#endif //SERIALPORT_MAP_THREAD_H
